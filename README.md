## cog_publisher_node  
calculate center of gravity from /robot_description parameters and /tf topic  

| *master* |
|----------|
|[![Build Status](https://travis-ci.org/OUXT-Polaris/cog_publisher.svg?branch=master)](https://travis-ci.org/OUXT-Polaris/cog_publisher)|

#### topics and parameters  

##### parameters  
- /robot_description  
type : string     
- ~publish_frame  
type : string  
default : base_link   
- ~publish_rate  
type : int  
default: 50  

##### subscribe topics  
- /tf  
message_type :
[tf2_msgs/TFMessage](http://docs.ros.org/jade/api/tf2_msgs/html/msg/TFMessage.html)  

##### publish topics  
- /cog/links  
message type : [sensor_msgs/PointCloud](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud.html)  
center of gravity in each link
- /cog/robot  
message type :
[geometry_msgs/PointStamped](http://docs.ros.org/jade/api/geometry_msgs/html/msg/PointStamped.html)  
center of gravity in whole robot  