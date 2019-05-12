// headers for ros
#include <ros/ros.h>

// headers in this package
#include <vision_msgs_visualization/detection_2d_visualizer.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "detection_2d_visualizer_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    Detection2DVisualizer visualizer = Detection2DVisualizer(nh,pnh);
    ros::spin();
    return 0;
}