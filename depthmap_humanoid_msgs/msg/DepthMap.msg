# This represents a 2.5D grid map, in which each cell represents depth informaption
# so it is a depth map

Header header

#MetaData for the map
nav_msgs/MapMetaData info
# (deprecated) The map data is the hight value in meters, in row-major order, starting with (0,0).
# The map data (pixel values) as a sensor Image value to store custom variable type pixels
# (custom set the type to sensor_msgs::Image_encoding::TYPE_32FC1)
sensor_msgs/Image map
