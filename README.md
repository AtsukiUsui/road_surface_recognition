# road_surface_recognition pkg

This ROS pkg provides the method making occupancy grid map that express lawn place data as occupancy.  

## Images

For example, if you use the map (Fig. 1), you can make the following map (Fig. 2). 

<img src="https://github.com/KentaKubota/road_surface_recognition/blob/master/images/map.png" title="Fig.1 map" width="600px" alt="Fig.1 map">

<img src="https://github.com/KentaKubota/road_surface_recognition/blob/master/images/lawnMap.png" title="Fig.2 lawnMap" width="600px" alt="Fig.2 lawnMap">

## Preparation

Please install `laser_geometry` pkg in your catkin_ws/src/ because this pkg uses laser_geometry pkg function.
<https://github.com/ros-perception/laser_geometry.git>

## How to use

### NEW
'''
roslaunch road_surface_recognition build_reflection_mapping.launch map_file:=/home/atsuki/lab_ws/src/experiment/2023-07-10-1/map/1_2023-07-10-11-42-50.yaml  rosbag:=true rosbag_rate:=1 rosbag_filename:=/home/atsuki/lab_ws/src/experiment/2023-07-10-1/1_2023-07-10-11-42-50.bag
'''
'''
rosservice call /up_map
'''

### old
```
roslaunch road_surface_recognition build_reflection_mapping.launch
```
```
rosrun map_server map_server ~/lab_ws/src/road_surface_recognition/test/map.yaml
```
```
rosbag play ~/lab_ws/src/road_surface_recognition/test/bagFile.bag
```
[ INFO] [1681377668.192825764]: Function in pointcloudCallback
[ INFO] [1681377668.192993578]: map_update_cell
```
rosservice call /up_map
```
[ INFO] [1681378656.559484596]: map_update_cell2
[ INFO] [1681378658.009742985]: Published an OccupancyGrid data of which topic name is occupancyGrid
```
rosrun map_server map_saver -f mapFileName map:=lawnOccupancyGrid
```



