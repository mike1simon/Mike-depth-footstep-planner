/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Carnegie Mellon University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <cstdio>
#include <ctime>
#include <sstream>

#include <sbpl_edit/Depth2Dgridsearch.h>
//?? FOR TESTING
#include <ros/ros.h>
void dest(){

}
namespace sbpl_edit {

void dest(){

}

using namespace std;

// Creating custom getCost functions that will combine multiple cells in the main grid
// into a single cell in the 2D grid.
// Using templates for common cases so the compiler can generate faster code
template<int N>
unsigned char getCostT(unsigned char** FeasibilityGrid2D, int x, int y, int /*resample*/)
{
    if(N == 1) // the compiler will optimize out this if statement
    {
        return FeasibilityGrid2D[x][y];
    }
    else
    {
        x *= N;
        y *= N;
        unsigned char maximum = FeasibilityGrid2D[x][y];
        for(int dy=0; dy<N; dy++)
            for(int dx=0; dx<N; dx++)
                maximum = __max(maximum, FeasibilityGrid2D[x+dx][y+dy]);

        return maximum;
    }
}

// fallback for unforseen cases
unsigned char getCostN(unsigned char **FeasibilityGrid2D, int x, int y, int resample)
{
    x *= resample;
    y *= resample;
    unsigned char maximum = FeasibilityGrid2D[x][y];
    for(int dy=0; dy<resample; dy++)
        for(int dx=0; dx<resample; dx++)
            maximum = __max(maximum, FeasibilityGrid2D[x+dx][y+dy]);

    return maximum;
}
unsigned char getCostDownSampled(unsigned char** FeasibilityGrid2D, int x, int y, int /*resample*/)
{
    return FeasibilityGrid2D[x][y];
}

//---------------------initialization and destruction routines--------------------------------------------------------
DEPTH2DGridSearch::DEPTH2DGridSearch(int width_x, int height_y, float cellsize_m,
 float Depth_resolution, int downsample, double maximumStepHight, double maximumStepReachability,
  double max_attitude, double min_attitude,  double MAX_RESOLUTION, int initial_dynamic_bucket_size)
{
    iteration_ = 0;
    searchStates2D_ = NULL;
    downsample_ = __max(1, downsample);

    width_ = width_x / downsample;
    height_ = height_y / downsample;
    cellSize_m_ = cellsize_m * downsample;
    Depth_resolution_ = Depth_resolution;
    maximumStepReachability_ = static_cast<int>(maximumStepReachability / cellSize_m_);
    max_attitude_ = max_attitude;
    min_attitude_ = min_attitude;
    MAX_RESOLUTION_ = MAX_RESOLUTION;
    Depth_threshold_ = maximumStepHight;
    // choose appropriate hard-coded getCost for resample amount
    switch(downsample_)
    {
    case 1: getCost = &getCostT<1>; break;
    case 2: getCost = &getCostT<2>; break;
    case 3: getCost = &getCostT<3>; break;
    case 4: getCost = &getCostT<4>; break;
    case 5: getCost = &getCostT<5>; break;
    case 6: getCost = &getCostT<6>; break;
    case 7: getCost = &getCostT<7>; break;
    case 8: getCost = &getCostT<8>; break;
    case 9: getCost = &getCostT<9>; break;
    case 10: getCost = &getCostT<10>; break;
    default:
        getCost = &getCostN;
    }

    startX_ = -1;
    startY_ = -1;
    goalX_ = -1;
    goalY_ = -1;

    largestcomputedoptf_ = 0;

    //compute dx, dy, dxintersects and dyintersects arrays
    computedxy();

    term_condition_usedlast = SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS;

    //allocate memory
    OPEN2D_ = new CIntHeap(width_ * height_);
    if (!createSearchStates2D()) {
        throw SBPL_Exception("ERROR: failed to create searchstatespace2D");
    }

    //by default, OPEN is implemented as heap
    OPEN2DBLIST_ = NULL;
    OPENtype_ = SBPL_2DGRIDSEARCH_OPENTYPE_HEAP;

    initial_dynamic_bucket_size_ = initial_dynamic_bucket_size;
}

bool DEPTH2DGridSearch::setOPENdatastructure(SBPL_2DGRIDSEARCH_OPENTYPE OPENtype)
{
    OPENtype_ = OPENtype;

    switch (OPENtype_) {
    case SBPL_2DGRIDSEARCH_OPENTYPE_HEAP:
        //this is the default, nothing else needs to be done
        break;
    case SBPL_2DGRIDSEARCH_OPENTYPE_SLIDINGBUCKETS:
        SBPL_PRINTF("setting OPEN2D data structure to sliding buckets\n");
        if (OPEN2DBLIST_ == NULL) {
            //create sliding buckets
            //compute max distance for edge
            int maxdistance = 0;
            for (int dind = 0; dind < SBPL_2DGRIDSEARCH_NUMOF2DDIRS; dind++) {
                maxdistance = __max(maxdistance, dxy_distance_mm_[dind]);
            }
            int bucketsize = __max(1000, this->width_ + this->height_);
            int numofbuckets = 255 * maxdistance;
            SBPL_PRINTF("creating sliding bucket-based OPEN2D %d buckets, each bucket of size %d ...", numofbuckets,
                        bucketsize);
            OPEN2DBLIST_ = new CSlidingBucket(numofbuckets, bucketsize, initial_dynamic_bucket_size_);
            SBPL_PRINTF("done\n");
        }
        //delete other data structures
        if (OPEN2D_ != NULL) {
            OPEN2D_->makeemptyheap();
            delete OPEN2D_;
            OPEN2D_ = NULL;
        }

        break;
    default:
        std::stringstream ss("ERROR: unknown data structure type = ");
        ss << OPENtype_ << " for OPEN2D";
        throw SBPL_Exception(ss.str());
    };

    return true;
}

bool DEPTH2DGridSearch::createSearchStates2D(void)
{
    int x, y;

    if (searchStates2D_ != NULL) {
        SBPL_ERROR("ERROR: We already have a non-NULL search states array\n");
        return false;
    }

    searchStates2D_ = new DEPTH_2DGridSearchState*[width_];
    for (x = 0; x < width_; x++) {
        searchStates2D_[x] = new DEPTH_2DGridSearchState[height_];
        for (y = 0; y < height_; y++) {
            searchStates2D_[x][y].iterationaccessed = iteration_;
            searchStates2D_[x][y].x = x;
            searchStates2D_[x][y].y = y;
            initializeSearchState2D(&searchStates2D_[x][y]);
        }
    }
    return true;
}

inline void DEPTH2DGridSearch::initializeSearchState2D(DEPTH_2DGridSearchState* state2D)
{
    state2D->g = INFINITECOST;
    state2D->heapindex = 0;
    state2D->iterationaccessed = iteration_;
}

void DEPTH2DGridSearch::destroy()
{
    // destroy the OPEN list:
    if (OPEN2D_ != NULL) {
        OPEN2D_->makeemptyheap();
        delete OPEN2D_;
        OPEN2D_ = NULL;
    }

    // destroy the 2D states:
    if (searchStates2D_ != NULL) {
        for (int x = 0; x < width_; x++) {
            delete[] searchStates2D_[x];
        }
        delete[] searchStates2D_;
        searchStates2D_ = NULL;
    }

    if (OPEN2DBLIST_ != NULL) {
        delete OPEN2DBLIST_;
        OPEN2DBLIST_ = NULL;
    }
}

//void sbpl_edit::DEPTH2DGridSearch::des(){
//  // destroy the OPEN list:
//  if (OPEN2D_ != NULL) {
//      OPEN2D_->makeemptyheap();
//      delete OPEN2D_;
//      OPEN2D_ = NULL;
//  }

//  // destroy the 2D states:
//  if (searchStates2D_ != NULL) {
//      for (int x = 0; x < width_; x++) {
//          delete[] searchStates2D_[x];
//      }
//      delete[] searchStates2D_;
//      searchStates2D_ = NULL;
//  }

//  if (OPEN2DBLIST_ != NULL) {
//      delete OPEN2DBLIST_;
//      OPEN2DBLIST_ = NULL;
//  }
//}
void DEPTH2DGridSearch::computedxy()
{
    //initialize some constants for 2D search
    dx_[0] = 1;
    dy_[0] = 1;
    dx0intersects_[0] = -1;
    dy0intersects_[0] = -1;
    dx_[1] = 1;
    dy_[1] = 0;
    dx0intersects_[1] = -1;
    dy0intersects_[1] = -1;
    dx_[2] = 1;
    dy_[2] = -1;
    dx0intersects_[2] = -1;
    dy0intersects_[2] = -1;
    dx_[3] = 0;
    dy_[3] = 1;
    dx0intersects_[3] = -1;
    dy0intersects_[3] = -1;
    dx_[4] = 0;
    dy_[4] = -1;
    dx0intersects_[4] = -1;
    dy0intersects_[4] = -1;
    dx_[5] = -1;
    dy_[5] = 1;
    dx0intersects_[5] = -1;
    dy0intersects_[5] = -1;
    dx_[6] = -1;
    dy_[6] = 0;
    dx0intersects_[6] = -1;
    dy0intersects_[6] = -1;
    dx_[7] = -1;
    dy_[7] = -1;
    dx0intersects_[7] = -1;
    dy0intersects_[7] = -1;

    //Note: these actions have to be starting at 8 and through 15, since they
    //get multiplied correspondingly in Dijkstra's search based on index
#if SBPL_2DGRIDSEARCH_NUMOF2DDIRS == 16
    dx_[8] = 2; dy_[8] = 1;
    dx0intersects_[8] = 1; dy0intersects_[8] = 0; dx1intersects_[8] = 1; dy1intersects_[8] = 1;
    dx_[9] = 1; dy_[9] = 2;
    dx0intersects_[9] = 0; dy0intersects_[9] = 1; dx1intersects_[9] = 1; dy1intersects_[9] = 1;
    dx_[10] = -1; dy_[10] = 2;
    dx0intersects_[10] = 0; dy0intersects_[10] = 1; dx1intersects_[10] = -1; dy1intersects_[10] = 1;
    dx_[11] = -2; dy_[11] = 1;
    dx0intersects_[11] = -1; dy0intersects_[11] = 0; dx1intersects_[11] = -1; dy1intersects_[11] = 1;
    dx_[12] = -2; dy_[12] = -1;
    dx0intersects_[12] = -1; dy0intersects_[12] = 0; dx1intersects_[12] = -1; dy1intersects_[12] = -1;
    dx_[13] = -1; dy_[13] = -2;
    dx0intersects_[13] = 0; dy0intersects_[13] = -1; dx1intersects_[13] = -1; dy1intersects_[13] = -1;
    dx_[14] = 1; dy_[14] = -2;
    dx0intersects_[14] = 0; dy0intersects_[14] = -1; dx1intersects_[14] = 1; dy1intersects_[14] = -1;
    dx_[15] = 2; dy_[15] = -1;
    dx0intersects_[15] = 1; dy0intersects_[15] = 0; dx1intersects_[15] = 1; dy1intersects_[15] = -1;
#endif

    //compute distances
    for (int dind = 0; dind < SBPL_2DGRIDSEARCH_NUMOF2DDIRS; dind++) {

        if (dx_[dind] != 0 && dy_[dind] != 0) {
            if (dind <= 7)
                //the cost of a diagonal move in millimeters
                dxy_distance_mm_[dind] = (int)(cellSize_m_ * 1414);
            else
                //the cost of a move to 1,2 or 2,1 or so on in millimeters
                dxy_distance_mm_[dind] = (int)(cellSize_m_ * 2236);
        }
        else
            dxy_distance_mm_[dind] = (int)(cellSize_m_ * 1000); //the cost of a horizontal move in millimeters
    }
}

//--------------------------------------------------------------------------------------------------------------------

//-----------------------------------------debugging functions----------------------------------------------------------

void DEPTH2DGridSearch::printvalues()
{

}

//--------------------------------------------------------------------------------------------------------------------

//-----------------------------------------main functions--------------------------------------------------------------

bool DEPTH2DGridSearch::search(unsigned char** FeasibilityGrid2D, float** Depth2D, unsigned char obsthresh, int startx_c, int starty_c, int goalx_c,
                              int goaly_c, unsigned int maximum_allowed_hieght, SBPL_2DGRIDSEARCH_TERM_CONDITION termination_condition)
{
    startx_c /= downsample_;
    starty_c /= downsample_;

    goalx_c /= downsample_;
    goaly_c /= downsample_;

    switch (OPENtype_) {
    case SBPL_2DGRIDSEARCH_OPENTYPE_HEAP:
        return DEPTH2DGridSearch::search_withheap(FeasibilityGrid2D, Depth2D, obsthresh, startx_c, starty_c, goalx_c, goaly_c,
                                                 maximum_allowed_hieght, termination_condition);
        break;
    case SBPL_2DGRIDSEARCH_OPENTYPE_SLIDINGBUCKETS:
        return DEPTH2DGridSearch::search_withslidingbuckets(FeasibilityGrid2D, Depth2D, obsthresh, startx_c, starty_c, goalx_c, goaly_c,
                                                           maximum_allowed_hieght, termination_condition);
        break;
    default:
        std::stringstream ss("ERROR: unknown data structure type = ");
        ss << OPENtype_ << " for OPEN2D";
        throw SBPL_Exception(ss.str());
    };
    return false;
}

bool DEPTH2DGridSearch::search_withheap(unsigned char** FeasibilityGrid2D, float** Depth2D, unsigned char obsthresh, int startx_c, int starty_c,
                                       int goalx_c, int goaly_c, unsigned int maximum_allowed_hieght, SBPL_2DGRIDSEARCH_TERM_CONDITION termination_condition)
{
    DEPTH_2DGridSearchState *searchExpState = NULL;
    DEPTH_2DGridSearchState *searchPredState = NULL;
    int numofExpands = 0;
    int key;

    //get the current time
    clock_t starttime = clock();

    //closed = 0
    iteration_++;

    //init start and goal coordinates
    startX_ = startx_c;
    startY_ = starty_c;
    goalX_ = goalx_c;
    goalY_ = goaly_c;

    //clear the heap
    OPEN2D_->makeemptyheap();

    //set the term. condition
    term_condition_usedlast = termination_condition;
    //check the validity of start/goal
    if (!withinMap(startx_c, starty_c) || !withinMap(goalx_c, goaly_c)) {
        SBPL_ERROR("ERROR: grid2Dsearch is called on invalid start (%d %d) or goal(%d %d)\n", startx_c, starty_c,
                   goalx_c, goaly_c);
        return false;
    }

    // initialize the start and goal states
    searchExpState = &searchStates2D_[startX_][startY_];
    initializeSearchState2D(searchExpState);
    initializeSearchState2D(&searchStates2D_[goalx_c][goaly_c]);
    DEPTH_2DGridSearchState* search2DGoalState = &searchStates2D_[goalx_c][goaly_c];

    //seed the search
    searchExpState->setCost(0);
    // !! Debugging change the hard coded 20 to a parameter
    searchExpState->setReachability(maximumStepReachability_);

    key = searchExpState->getCost();
    if (termination_condition == SBPL_2DGRIDSEARCH_TERM_CONDITION_OPTPATHFOUND)
        //use h-values only if we are NOT computing all state values
        key = key + SBPL_2DGRIDSEARCH_HEUR2D(startX_, startY_);

    OPEN2D_->insertheap(searchExpState, key);

    //set the termination condition
    float term_factor = 0.0;
    switch (termination_condition) {
    case SBPL_2DGRIDSEARCH_TERM_CONDITION_OPTPATHFOUND:
        term_factor = 1;
        break;
    case SBPL_2DGRIDSEARCH_TERM_CONDITION_20PERCENTOVEROPTPATH:
        term_factor = (float)(1.0 / 1.2);
        break;
    case SBPL_2DGRIDSEARCH_TERM_CONDITION_TWOTIMESOPTPATH:
        term_factor = 0.5;
        break;
    case SBPL_2DGRIDSEARCH_TERM_CONDITION_THREETIMESOPTPATH:
        term_factor = (float)(1.0 / 3.0);
        break;
    case SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS:
        term_factor = 0.0;
        break;
    default:
        SBPL_ERROR("ERROR: incorrect termination factor for grid2Dsearch\n");
        term_factor = 0.0;
    };

    char *pbClosed = (char*)calloc(1, width_ * height_);

    //the main repetition of expansions
    while (!OPEN2D_->emptyheap() &&
           __min(INFINITECOST, search2DGoalState->g) > term_factor * OPEN2D_->getminkeyheap())
    {
        //get the next state for expansion
        searchExpState = (DEPTH_2DGridSearchState*)OPEN2D_->deleteminheap();
        numofExpands++;
        
        int exp_x = searchExpState->x;
        int exp_y = searchExpState->y;

        //close the state
        pbClosed[exp_x + width_ * exp_y] = 1;

        //iterate over successors
        int expcost =  getCostDownSampled(FeasibilityGrid2D, exp_x, exp_y, downsample_);
        for (int dir = 0; dir < SBPL_2DGRIDSEARCH_NUMOF2DDIRS; dir++) {
            int newx = exp_x + dx_[dir];
            int newy = exp_y + dy_[dir];

            //make sure it is inside the map and has no obstacle
            if (!withinMap(newx, newy)) continue;
            
            // ** might change this to allow changing cost or reachability
            //make sure it is not proccessed before
            if (pbClosed[newx + width_ * newy] == 1) continue;

            //compute the cost (the cost can't be less than current state cost)
            //in this implementation its either 0 or 255 so we just check if it is an obstacle make it 255
            int mapcost = __max(getCostDownSampled(FeasibilityGrid2D, newx, newy, downsample_), expcost);

#if SBPL_2DGRIDSEARCH_NUMOF2DDIRS > 8
            if(dir > 7) {
                //check two more cells through which the action goes
                mapcost = __max(mapcost, getCostDownSampled(FeasibilityGrid2D, exp_x + dx0intersects_[dir], exp_y + dy0intersects_[dir], downsample_));
                mapcost = __max(mapcost, getCostDownSampled(FeasibilityGrid2D, exp_x + dx1intersects_[dir], exp_y + dy1intersects_[dir], downsample_));
            }
#endif


            // !! Debugging 
            // // ? Change here to reduce the length of Reachability when obstacle encountered
            // if (mapcost >= obsthresh) //obstacle encountered
            // continue;

            // check Reachability value when obstacle encountered is it reachable (process it)
            // is it not reachable ignore it
            if (mapcost >= obsthresh && !searchExpState->isValid()) //obstacle encountered
                // don't take this state into account bcz its not reachable
                continue;
            //get the predecessor
            searchPredState = &searchStates2D_[newx][newy];

            // handel the not valid but still interesting states and reduce there Reachability
            if(mapcost >= obsthresh)
                searchPredState->setReachability(searchExpState->getReachability() - 1);
            else // in case of the PredState is Valid and the last state is not
                 // (reset the reachability variable to its initial state)
                // !! Debugging change the hard coded 20 to a parameter
                searchPredState->setReachability(maximumStepReachability_);
            
            // check hight difference if it is not reachable don't process it.
            float depth_difference = Depth2D[newx][newy] - Depth2D[exp_x][exp_y];
            if(std::abs(depth_difference) > Depth_threshold_)
                continue;

            // * the cost for now is either 0 or 255 (can be changed)
            // * this means that the new int cost = 1 * dxy_distance_mm_ (distance from the current expState)
            // // ** this could be changed to add in the reduced Reachability length 
            // int cost = (mapcost + 1) * dxy_distance_mm_[dir];
            // ** this could be changed to add in the reduced Reachability length into the cost 
            // !! Debugging test the cost of 2d search
            // int cost = (0.8 *(maximumStepReachability_ - searchPredState->getReachability())/maximumStepReachability_ +1) * dxy_distance_mm_[dir];
            int cost = ((maximumStepReachability_ - searchPredState->getReachability())/maximumStepReachability_ +1) * dxy_distance_mm_[dir];

            // * update the cost of the predecessor and add it to the heap
            //update predecessor if necessary
            if (searchPredState->iterationaccessed != iteration_ || searchPredState->getCost() > cost + searchExpState->getCost()) {
                searchPredState->iterationaccessed = iteration_;
                // * the predecessor new cost is either INFINITECOST or the cost of locating to it 
                // * from the current state + the current expanded state cost
                searchPredState->setCost( __min(INFINITECOST, cost + searchExpState->getCost()));
                key = searchPredState->getCost();
                if (termination_condition == SBPL_2DGRIDSEARCH_TERM_CONDITION_OPTPATHFOUND)
                    //use h-values only if we are NOT computing all state values
                    key = key + SBPL_2DGRIDSEARCH_HEUR2D(searchPredState->x, searchPredState->y);
                // * if the predecessor wasn't put in the heap before it will have heapindex of 0
                if (searchPredState->heapindex == 0) // * in this case insert it to heap
                    OPEN2D_->insertheap(searchPredState, key);
                else // * but if it was allready exiting just update it
                    OPEN2D_->updateheap(searchPredState, key);
            }
        } //over successors
    }//while

    //set lower bounds for the remaining states
    if (!OPEN2D_->emptyheap())
        largestcomputedoptf_ = OPEN2D_->getminkeyheap();
    else
        largestcomputedoptf_ = INFINITECOST;

    free(pbClosed);

    SBPL_PRINTF( "# of expands during 2dgridsearch=%d time=%d msecs 2Dsolcost_inmm=%d "
                "largestoptfval=%d (start=%d %d goal=%d %d)\n",
                numofExpands, (int)(((clock() - starttime) / (double)CLOCKS_PER_SEC) * 1000),
                searchStates2D_[goalx_c][goaly_c].g, largestcomputedoptf_, startx_c, starty_c, goalx_c, goaly_c);

    return true;
}

//experimental version
//have only one copy of a state in the list, but the order of expansions is
//BFS with possible re-expansions. Thus the order can be maintained by FIFO
//queue implemented by list (requires though the implementation of the lastelement in the list)
bool DEPTH2DGridSearch::search_exp(unsigned char** FeasibilityGrid2D, float** Depth2D, unsigned char obsthresh, int startx_c, int starty_c,
                                  int goalx_c, int goaly_c,unsigned int maximum_allowed_hieght)
{
    DEPTH_2DGridSearchState *searchExpState = NULL;
    DEPTH_2DGridSearchState *searchPredState = NULL;
    int numofExpands = 0;
    CList OPEN2DLIST;

    //get the current time
    clock_t starttime = clock();

    //closed = 0
    iteration_++;

    //init start and goal coordinates
    startX_ = startx_c;
    startY_ = starty_c;
    goalX_ = goalx_c;
    goalY_ = goaly_c;

    //check the validity of start/goal
    if (!withinMap(startx_c, starty_c) || !withinMap(goalx_c, goaly_c)) {
        SBPL_ERROR("ERROR: grid2Dsearch is called on invalid start (%d %d) or goal(%d %d)\n", startx_c, starty_c,
                   goalx_c, goaly_c);
        return false;
    }

    // initialize the start and goal states
    searchExpState = &searchStates2D_[startX_][startY_];
    initializeSearchState2D(searchExpState);
    //no initialization for the goal state - it will reset iteration_ variable

    //seed the search
    searchExpState->g = 0;
    searchExpState->listelem[SBPL_2DSEARCH_OPEN_LIST_ID] = NULL;
    OPEN2DLIST.insertinfront(searchExpState, SBPL_2DSEARCH_OPEN_LIST_ID);

    //the main repetition of expansions
    while (!OPEN2DLIST.empty()) {
        //get the next state for expansion
        searchExpState = (DEPTH_2DGridSearchState*)OPEN2DLIST.getlast();
        OPEN2DLIST.remove(searchExpState, SBPL_2DSEARCH_OPEN_LIST_ID);
        numofExpands++;

        int exp_x = searchExpState->x;
        int exp_y = searchExpState->y;

        //iterate over successors
        for (int dir = 0; dir < SBPL_2DGRIDSEARCH_NUMOF2DDIRS; dir++) {
            int newx = exp_x + dx_[dir];
            int newy = exp_y + dy_[dir];

            //make sure it is inside the map and has no obstacle
            if (!withinMap(newx, newy)) continue;

            //compute the cost
            int mapcost = __max( getCostDownSampled(FeasibilityGrid2D, newx, newy, downsample_), getCostDownSampled(FeasibilityGrid2D, exp_x, exp_y, downsample_));

#if SBPL_2DGRIDSEARCH_NUMOF2DDIRS > 8
            if(dir > 7) {
                //check two more cells through which the action goes
                mapcost = __max(mapcost, getCostDownSampled(FeasibilityGrid2D, exp_x + dx0intersects_[dir], exp_y + dy0intersects_[dir], downsample_));
                mapcost = __max(mapcost, getCostDownSampled(FeasibilityGrid2D, exp_x + dx1intersects_[dir], exp_y + dy1intersects_[dir], downsample_));
            }
#endif

            if (mapcost >= obsthresh) //obstacle encountered
            continue;
            int cost = (mapcost + 1) * dxy_distance_mm_[dir];

            //get the predecessor
            searchPredState = &searchStates2D_[newx][newy];

            //clear the list
            if (searchPredState->iterationaccessed != iteration_) searchPredState->listelem[SBPL_2DSEARCH_OPEN_LIST_ID]
                = NULL;

            //update predecessor if necessary
            if (searchPredState->iterationaccessed != iteration_ || searchPredState->g > cost + searchExpState->g) {
                searchPredState->iterationaccessed = iteration_;
                searchPredState->g = __min(INFINITECOST, cost + searchExpState->g);

                if (searchPredState->g >= INFINITECOST) {
                    throw SBPL_Exception("ERROR: infinite g");
                }

                //put it into the list if not there already
                if (searchPredState->listelem[SBPL_2DSEARCH_OPEN_LIST_ID] == NULL)
                    OPEN2DLIST.insertinfront(searchPredState, SBPL_2DSEARCH_OPEN_LIST_ID);
                //otherwise, the state remains where it is but it now has a new g-value
            }
        } //over successors
    }//while

    //set lower bounds for the remaining states - none left since we exhausted the whole list
    largestcomputedoptf_ = INFINITECOST;

    SBPL_PRINTF( "# of expands during 2dgridsearch=%d time=%d msecs 2Dsolcost_inmm=%d "
                "largestoptfval=%d (start=%d %d goal=%d %d)\n",
                numofExpands, (int)(((clock() - starttime) / (double)CLOCKS_PER_SEC) * 1000),
                searchStates2D_[goalx_c][goaly_c].g, largestcomputedoptf_, startx_c, starty_c, goalx_c, goaly_c);

    return false;
}

//experimental version
//OPEN list is implemented as a bucket list
bool DEPTH2DGridSearch::search_withbuckets(unsigned char** FeasibilityGrid2D, float** Depth2D,unsigned char obsthresh, int startx_c, int starty_c,
                                          int goalx_c, int goaly_c, unsigned int maximum_allowed_hieght)
{
    DEPTH_2DGridSearchState *searchExpState = NULL;
    DEPTH_2DGridSearchState *searchPredState = NULL;
    int numofExpands = 0;

    //get the current time
    clock_t starttime = clock();

    //int max_bucketed_priority = obsthresh*10*__max(this->width_, this->height_);
    int max_bucketed_priority = (int)(0.1 * obsthresh * dxy_distance_mm_[0] * __max(this->width_, this->height_));
    SBPL_PRINTF("bucket-based OPEN2D has up to %d bucketed priorities, the rest will be unsorted\n",
                max_bucketed_priority);
    CBucket OPEN2DBLIST(0, max_bucketed_priority);

    SBPL_PRINTF("OPEN2D allocation time=%d msecs\n", (int)(((clock() - starttime) / (double)CLOCKS_PER_SEC) * 1000));

    //closed = 0
    iteration_++;

    //init start and goal coordinates
    startX_ = startx_c;
    startY_ = starty_c;
    goalX_ = goalx_c;
    goalY_ = goaly_c;

    //check the validity of start/goal
    if (!withinMap(startx_c, starty_c) || !withinMap(goalx_c, goaly_c)) {
        SBPL_ERROR("ERROR: grid2Dsearch is called on invalid start (%d %d) or goal(%d %d)\n", startx_c, starty_c,
                   goalx_c, goaly_c);
        return false;
    }

    // initialize the start and goal states
    searchExpState = &searchStates2D_[startX_][startY_];
    initializeSearchState2D(searchExpState);
    //no initialization for the goal state - it will reset iteration_ variable

    //seed the search
    searchExpState->g = 0;
    searchExpState->heapindex = -1;
    OPEN2DBLIST.insert(searchExpState, searchExpState->g);

    //the main repetition of expansions
    while (!OPEN2DBLIST.empty()) {
        //get the next state for expansion
        searchExpState = (DEPTH_2DGridSearchState*)OPEN2DBLIST.popminelement();
        numofExpands++;

        int exp_x = searchExpState->x;
        int exp_y = searchExpState->y;

        //iterate over successors
        for (int dir = 0; dir < SBPL_2DGRIDSEARCH_NUMOF2DDIRS; dir++) {
            int newx = exp_x + dx_[dir];
            int newy = exp_y + dy_[dir];

            //make sure it is inside the map and has no obstacle
            if (!withinMap(newx, newy)) continue;

            //compute the cost
            int mapcost = __max( getCostDownSampled(FeasibilityGrid2D, newx, newy, downsample_), getCostDownSampled(FeasibilityGrid2D, exp_x, exp_y, downsample_));

#if SBPL_2DGRIDSEARCH_NUMOF2DDIRS > 8
            if(dir > 7) {
                //check two more cells through which the action goes
                mapcost = __max(mapcost, getCostDownSampled(FeasibilityGrid2D, exp_x + dx0intersects_[dir], exp_y + dy0intersects_[dir], downsample_));
                mapcost = __max(mapcost, getCostDownSampled(FeasibilityGrid2D, exp_x + dx1intersects_[dir], exp_y + dy1intersects_[dir], downsample_));
            }
#endif

            if (mapcost >= obsthresh) //obstacle encountered
            continue;
            int cost = (mapcost + 1) * dxy_distance_mm_[dir];

            //get the predecessor
            searchPredState = &searchStates2D_[newx][newy];

            //clear the element from OPEN
            if (searchPredState->iterationaccessed != iteration_) searchPredState->heapindex = -1;

            //update predecessor if necessary
            if (searchPredState->iterationaccessed != iteration_ || searchPredState->g > cost + searchExpState->g) {
                int oldstatepriority = searchPredState->g;
                searchPredState->iterationaccessed = iteration_;
                searchPredState->g = __min(INFINITECOST, cost + searchExpState->g);

                if (searchPredState->g >= INFINITECOST) {
                    throw SBPL_Exception("ERROR: infinite g");
                }

                //put it into the list if not there already
                if (searchPredState->heapindex != -1) OPEN2DBLIST.remove(searchPredState, oldstatepriority);
                OPEN2DBLIST.insert(searchPredState, searchPredState->g);
            }
        } //over successors
    }//while

    //set lower bounds for the remaining states
    if (!OPEN2DBLIST.empty())
        largestcomputedoptf_ = OPEN2DBLIST.getminpriority();
    else
        largestcomputedoptf_ = INFINITECOST;

    SBPL_PRINTF( "# of expands during 2dgridsearch=%d time=%d msecs 2Dsolcost_inmm=%d "
                "largestoptfval=%d bucketassortedarraymaxsize=%d (start=%d %d goal=%d %d)\n",
                numofExpands, (int)(((clock() - starttime) / (double)CLOCKS_PER_SEC) * 1000),
                searchStates2D_[goalx_c][goaly_c].g, largestcomputedoptf_, OPEN2DBLIST.maxassortedpriorityVsize,
                startx_c, starty_c, goalx_c, goaly_c);

    return false;
}

//experimental version
//OPEN list is implemented as a sliding bucket list
bool DEPTH2DGridSearch::search_withslidingbuckets(unsigned char** FeasibilityGrid2D,float** Depth2D, unsigned char obsthresh, int startx_c,
                                                 int starty_c, int goalx_c, int goaly_c,
                                                 unsigned int maximum_allowed_hieght, SBPL_2DGRIDSEARCH_TERM_CONDITION termination_condition)
{
    DEPTH_2DGridSearchState *searchExpState = NULL;
    DEPTH_2DGridSearchState *searchPredState = NULL;
    int numofExpands = 0;

#if DEBUG
#ifndef ROS
    const char* f2dgriddebug = "2dgriddebug.txt";
#endif
    FILE* f2Dsearch = SBPL_FOPEN(f2dgriddebug, "w");
#endif

    //closed = 0
    iteration_++;

    //init start and goal coordinates
    startX_ = startx_c;
    startY_ = starty_c;
    goalX_ = goalx_c;
    goalY_ = goaly_c;

    //check the validity of start/goal
    if (!withinMap(startx_c, starty_c) || !withinMap(goalx_c, goaly_c)) {
        SBPL_ERROR("ERROR: grid2Dsearch is called on invalid start (%d %d) or goal(%d %d)\n", startx_c, starty_c,
                   goalx_c, goaly_c);
#if DEBUG
        SBPL_FCLOSE(f2Dsearch);
#endif
        return false;
    }

    //reset OPEN
    OPEN2DBLIST_->reset();

    //set the term. condition
    term_condition_usedlast = termination_condition;

    // initialize the start and goal states
    searchExpState = &searchStates2D_[startX_][startY_];
    initializeSearchState2D(searchExpState);
    initializeSearchState2D(&searchStates2D_[goalx_c][goaly_c]);
    DEPTH_2DGridSearchState* search2DGoalState = &searchStates2D_[goalx_c][goaly_c];

    //seed the search
    searchExpState->g = 0;
    OPEN2DBLIST_->insert(searchExpState, searchExpState->g);

    //set the termination condition
    float term_factor = 0.0;
    switch (termination_condition) {
    case SBPL_2DGRIDSEARCH_TERM_CONDITION_OPTPATHFOUND:
        term_factor = 1;
        break;
    case SBPL_2DGRIDSEARCH_TERM_CONDITION_20PERCENTOVEROPTPATH:
        term_factor = (float)(1.0 / 1.2);
        break;
    case SBPL_2DGRIDSEARCH_TERM_CONDITION_TWOTIMESOPTPATH:
        term_factor = 0.5;
        break;
    case SBPL_2DGRIDSEARCH_TERM_CONDITION_THREETIMESOPTPATH:
        term_factor = (float)(1.0 / 3.0);
        break;
    case SBPL_2DGRIDSEARCH_TERM_CONDITION_ALLCELLS:
        term_factor = 0.0;
        break;
    default:
        SBPL_ERROR("ERROR: incorrect termination factor for grid2Dsearch\n");
        term_factor = 0.0;
    };

    //get the current time
    clock_t starttime = clock();

    char *pbClosed = (char*)calloc(1, width_ * height_);

    //the main repetition of expansions
    SBPL_PRINTF("2D search with sliding buckets and term_factor=%.3f\n", term_factor);
    int prevg = 0;
    while (!OPEN2DBLIST_->empty() && search2DGoalState->g > term_factor * OPEN2DBLIST_->getminkey()) {
#if DEBUG
        SBPL_FPRINTF(f2Dsearch, "currentminelement_priority before pop=%d\n", OPEN2DBLIST_->getminkey());
#endif

        //get the next state for expansion
        searchExpState = (DEPTH_2DGridSearchState*)OPEN2DBLIST_->popminelement();

        if (searchExpState->g > OPEN2DBLIST_->getminkey()) {
            SBPL_ERROR("ERROR: g=%d for state %d %d but minpriority=%d (prevg=%d)\n", searchExpState->g,
                       searchExpState->x, searchExpState->y, OPEN2DBLIST_->getminkey(), prevg);
        }
        prevg = searchExpState->g;

        int exp_x = searchExpState->x;
        int exp_y = searchExpState->y;

        //close the state if it wasn't yet
        if (pbClosed[exp_x + width_ * exp_y] == 1) continue;
        pbClosed[exp_x + width_ * exp_y] = 1;

#if DEBUG
        SBPL_FPRINTF(f2Dsearch, "expanding state <%d %d> with g=%d "
                     "(currentminelement_priority=%d, currentfirstbucket_bindex=%d, currentfirstbucket_priority=%d)\n",
                     searchExpState->x, searchExpState->y, searchExpState->g, OPEN2DBLIST_->getminkey(),
                     OPEN2DBLIST_->currentfirstbucket_bindex, OPEN2DBLIST_->currentfirstbucket_priority);
#endif

        //expand
        numofExpands++;

        //iterate over successors
        int expcost = getCostDownSampled(FeasibilityGrid2D, exp_x, exp_y, downsample_);
        for (int dir = 0; dir < SBPL_2DGRIDSEARCH_NUMOF2DDIRS; dir++) {
            int newx = exp_x + dx_[dir];
            int newy = exp_y + dy_[dir];

            //make sure it is inside the map and has no obstacle
            if (!withinMap(newx, newy)) continue;

            if (pbClosed[newx + width_ * newy] == 1) continue;

            //compute the cost
            int mapcost = __max( getCostDownSampled(FeasibilityGrid2D, newx, newy, downsample_), expcost);

#if SBPL_2DGRIDSEARCH_NUMOF2DDIRS > 8
            if(dir > 7) {
                //check two more cells through which the action goes
                mapcost = __max(mapcost, getCostDownSampled(FeasibilityGrid2D, exp_x + dx0intersects_[dir], exp_y + dy0intersects_[dir], downsample_));
                mapcost = __max(mapcost, getCostDownSampled(FeasibilityGrid2D, exp_x + dx1intersects_[dir], exp_y + dy1intersects_[dir], downsample_));
            }
#endif

            if (mapcost >= obsthresh) //obstacle encountered
            continue;
            int cost = (mapcost + 1) * dxy_distance_mm_[dir];

            //get the predecessor
            searchPredState = &searchStates2D_[newx][newy];

            //update predecessor if necessary
            if (searchPredState->iterationaccessed != iteration_ || searchPredState->g > cost + searchExpState->g) {
                searchPredState->iterationaccessed = iteration_;
                searchPredState->g = __min(INFINITECOST, cost + searchExpState->g);

#if DEBUG
                SBPL_FPRINTF(f2Dsearch, "inserting state <%d %d> with g=%d\n", searchPredState->x, searchPredState->y, searchPredState->g);
#endif

                //put it into the list
                OPEN2DBLIST_->insert(searchPredState, searchPredState->g);
            }
        } //over successors
    }//while

    //set lower bounds for the remaining states
    if (!OPEN2DBLIST_->empty())
        largestcomputedoptf_ = ((DEPTH_2DGridSearchState*)OPEN2DBLIST_->getminelement())->g;
    else
        largestcomputedoptf_ = INFINITECOST;

    //delete[] pbClosed;
    free(pbClosed);

    SBPL_PRINTF( "# of expands during 2dgridsearch=%d time=%d msecs 2Dsolcost_inmm=%d "
                "largestoptfval=%d (start=%d %d goal=%d %d)\n",
                numofExpands, (int)(((clock() - starttime) / (double)CLOCKS_PER_SEC) * 1000),
                searchStates2D_[goalx_c][goaly_c].g, largestcomputedoptf_, startx_c, starty_c, goalx_c, goaly_c);

#if DEBUG
    SBPL_FCLOSE(f2Dsearch);
#endif
    return true;
}
}
//----------------------------------------------------------------------------------------------------------------------

