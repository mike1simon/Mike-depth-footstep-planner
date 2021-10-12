# This represents a 2.5D grid map, in which each cell represents depth informaption
# so it is a GreyScaleMap16bit map

Header header

#MetaData for the map
nav_msgs/MapMetaData info
# the minimum attitude used in depthmap
# the maximum attitude used in depthmap
float32 minimum_attitude
float32 maximum_attitude
# The map data, in row-major order, starting with (0,0).
uint16[] data
