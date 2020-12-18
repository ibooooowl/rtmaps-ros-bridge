# RTMaps ROS Bridge



The RTMaps ROS bridge is a package of components available in Linux and providing the possibility to:

- subscribe to ROS topics in RTMaps and therefore retrieve ROS data streams in RTMaps
- publish RTMaps streams to ROS topic and therefore transfer data from RTMaps to ROS
- synchronize the RTMaps clocks on the ROS clock so that timestamps associated to samples in both environments can remain coherent.


## Specific instructions for displaying Markers messages:

In order to display visualization_msgs::Marker and visualization_msgs::MarkerArray messages received by the ROS Bridge Subscriber, the inputs of the 3DViewer component have to be properly configured. This feature is available since ROS Bridge subscriber >= 2.5.2.

As opposed to RVIZ (a ROS Viewer), the 3DViewer does not accumulate objects for display. This accumulation has been implemented in the ROS Bridge.




Here are the object types needed to be set up in the 3DViewer:

For each ROS Marker type, the 3DViewer object type has to be set up accordingly:

- Cube : RealObject, shape: Cube

- Sphere: RealObject, shape: Sphere

- Arrow : RealObject, shape: Pyramid (limited for now)

- Cylinder : RealObject, shape: Cube (limited for now)

- Line strip: Trajectory

- Line list: Lines

- Triangle list: Triangles


