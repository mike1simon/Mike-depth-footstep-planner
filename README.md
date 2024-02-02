# Footstep Planning for Humanoid Robots Using Perception-based Feasible Footsteps in Cluttered Environments
## By Mike Simon

This repository contains the source code and documentation for my Master's thesis submitted in fulfillment of the requirements for the Master’s Degree in Control and Robotics, Specialty Robotics. The research focuses on developing an enhanced footstep planning algorithm for humanoid robots navigating cluttered environments, leveraging neural networks to pre-analyze the terrain and improve the efficiency of the ARA* algorithm's search expansion.
## Abstract

This work introduces an improvement to the footstep planning algorithm by utilizing neural networks for a preliminary analysis of the environment, thereby accelerating the search expansion process of the ARA* algorithm. The methodology encompasses several stages, starting from analyzing the terrain map with deep learning and neural networks based on a modified U-Net architecture. This is followed by a 2D expansion algorithm that propagates through the terrain map, serving as a heuristic cost function for the footstep planning process. A significant contribution of this research is the adaptation of an existing software package to map the planned footsteps from a 2D environment to a 3D one using the Robot Operating System (ROS). The findings of this study offer a foundational step towards further advancements in path planning algorithms and environmental modeling for humanoid robots.
## Keywords:

Path Planning, Footstep Planning, Deep Learning, Environment Analysis, Humanoid Robots, Robotic Operating System (ROS), FCN

### Master Thesis:
you can download the Thesis (in Arabic) from the Higher Institute for Applied Sciences and Technology [website](https://hiast.edu.sy/en/Master-PhD-Theses) or [here PDF](https://hiast.edu.sy/sites/default/files/MasterPHD/62c2e34f9ad01.pdf)

### 
### Dependencies

This project relies on various ROS packages and external libraries to function properly. Below is a list of dependencies required to build and run the `depth_footstep_planner` package:

#### ROS Packages:

- `depthmap_humanoid_msgs`: Custom message definitions for depth maps in humanoid robotics (assumed to be a custom package not found in standard ROS distributions).
- `humanoid_nav_msgs`: Custom message definitions used in humanoid navigation (assumed to be a custom package not found in standard ROS distributions).
- `nav_msgs`: Contains common messages that are used to interact with the navigation stack.
- `visualization_msgs`: Contains messages for visualizing data such as markers in tools like RViz.

#### External Libraries and Additional Dependencies:

- **PyTorch**: 
For PyTorch, please follow the [official PyTorch installation guide](https://pytorch.org/get-started/locally/) to install it according to your system configuration and CUDA compatibility (required for UNET model).

- **Boost System**:

    ```bash
    sudo apt-get install libboost-system-dev
    ```

- **OpenCV (Open Source Computer Vision Library)**:
OpenCV is typically included in ROS desktop-full installations. If not, it can be installed using:
    ```bash
    sudo apt-get install libopencv-dev
    ```

- **SBPL**:

    The SBPL library might need to be installed from source if not available in your ROS distribution:

    ```bash
    git clone https://github.com/sbpl/sbpl.git
    cd sbpl
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
    ```


### Setup and Installation

Instructions for setting up the environment and installing dependencies:

```bash
# Clone the repository
git clone https://github.com/mike1simon/Mike-depth-footstep-planner.git

# Navigate to the repository
cd Mike-depth-footstep-planner

# Setup ROS environment (assuming ROS Noetic)
source /opt/ros/noetic/setup.bash

# Install required ROS packages and dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the packages
catkin_make
```

### Running the Footstep Planner

To run the footstep planner with a sample environment:

```bash
# Source the setup script
source devel/setup.bash

# Launch the simulation environment
roslaunch roslaunch footstep_planner footstep_planner_complete.launch
```

### License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.

### Acknowledgments
This work is built on the existing ros humanoid stack packages, more details about that including the reference papers exist [ros footstep_planner](http://wiki.ros.org/footstep_planner)

I would like to express my gratitude to my supervisors and the robotics department for their guidance and support throughout this research.
