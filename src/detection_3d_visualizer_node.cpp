// headers for ros
#include <ros/ros.h>

// headers in this package
#include <vision_msgs_visualization/detection_3d_visualizer.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "detection_3d_visualizer_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    Detection3DVisualizer visualizer = Detection3DVisualizer(nh,pnh);
    ros::spin();
    return 0;
}