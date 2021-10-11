# udacity-service-bot
## Project 5: Home Service Robot From Udacity Robotics Engeneer Nanodegree

## Projecyt Summary
### Mapping
You can test the SLAM algorithm by running the following script from /src/scripts folder
```
./test_slam.sh
```
This script allows you to control the robot my hand using the turtlebot_teleop package in one of the terminals.
The robot utilizes the Gmapping SLAM package to locate itself and to build a map of the surrounding area. This algorithm relies on the laser scan data from the LIDAR and the odometry data from robot's wheels. It is possible to connect the depth camera sensor as additional distance measuring tool, but it was not considered in this project.
Below you can see the difference between the map built by SLAM and the real map of the robot's enviroment:
![map_slam](https://user-images.githubusercontent.com/5613558/136739008-874fcb12-6710-4929-b673-187fdd6249ac.png)
![map_true](https://user-images.githubusercontent.com/5613558/136739029-b295b3d9-fdc6-4d24-93c5-ccc4e1ca8e89.png)
### Localization
You can test the AMCL localization algorithm by running the following script from /src/scripts folder
```
./test_navigation.sh
```
Then you can command robot to move around using Set2D Navigation Goal tool in Rviz. You will see that the particle set converges on the robot's pose really quick. This is because the world is asymetrtic and the walls configuration is unique. So when the robot receives the LIDAR data - it can quickly determine where it is located relative to the map generated by SLAM.
### Navigation
You can test the Navigation algorithm by running the following script from /src/scripts folder
```
./pick_objects.sh
```
This script will run pick_objects node to command the robot to move to the pickup zone first and then to the dropoff zone. You should see the corresponding messages in the terminal as the robot reaches its goals.
To perform navigation the robot utilized the MoveBase ROS package. It allows to set target poses for the robot to reach and implements path building algorithms to build a safe obstacle-avoiding path from the current position of the robot to the target pose with given coordinates and orientation.
### Virtual Markers
You can test the Navigation algorithm by running the following script from /src/scripts folder
```
./add_markers.sh
```
This script will demonstrate the virtual objects cappability of Rviz. First it will create the marker in the pickup zone and then delete it in the 5 seconds. After another 5 seconds the marker should apper in the dropoff zone.
It should be noted that Rviz provides a flexible tools for working with virtual objects. In this example it was unnecessary to manualy delete the marker - the first marker could be created with the lifetime of 5 seconds after which it would be deleted automaticly.
### Home Service
You can run the whole project with the following script from /src/scripts folder
```
./home_service.sh
```
This script will send the robot to the pickup zone to pick up the marker. After arrival you should see the marker disappearing in Rviz, simulating the robot picking up the package. After 5 seconds delay the robot will receive the drop zone coordinates and navigate to it. At the arrival the robot will drop the package.
This project utilized ROS service functions to implement the package pickup and dropoff simulation. To make this work without breaking the previous scripts two new nodes were created: add_markers/markers_srv and pick_objects/home_service.
#### add_markers/markers_srv
This node creates service messega packageAction.srv and provides the server for two services: /add_markers/pickup and /add_markers/dropoff. The first one is used to pick up the package, it will check if the robot is within certain distance of the requested package and then delete the marker on the pickup location. The second service checks if the robot is near the dropoff lication and creates a marker at current location of the robot.
#### pick_objects/home_service
This node utilizes MoveBase Client and the use of add_markers/pickup and /add_markers/dropoff services. First it commands the robot to move to the pickup zone. Then it attempts to pick up the package in the pickup zone (the service will check if the robot is close enough to the package to actually pick it up). Then, if the pickup was successeful, the robot moves to the dropoff zone. There it attempts to drop the package (the service will once again check if the robot is actually in the dropzone to do so). If successeful the "Delivery successeful!" message is displayed.
