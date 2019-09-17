#ifndef VISION_MSGS_VISUALIZATION_DETECTION_3D_VISUALIZER_H_INCLUDED
#define VISION_MSGS_VISUALIZATION_DETECTION_3D_VISUALIZER_H_INCLUDED

// Headers in ROS
#include <ros/ros.h>
#include <vision_msgs/VisionInfo.h>
#include <vision_msgs/Detection3DArray.h>
#include <vision_msgs/VisionInfo.h>
#include <vision_info_server/vision_info_parser.h>
#include <visualization_msgs/MarkerArray.h>

// Headers in STL
#include <random>

class Detection3DVisualizer
{
public:
    Detection3DVisualizer(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~Detection3DVisualizer();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    void detectionCallback(const vision_msgs::Detection3DArray::ConstPtr msg);
    void visionInfoCallback(const vision_msgs::VisionInfo::ConstPtr msg);
    boost::optional<visualization_msgs::MarkerArray> generateMarker(vision_msgs::Detection3DArray detection);
    std::string vision_info_topic_;
    std::string detection_topic_;
    ros::Subscriber detection_sub_;
    ros::Subscriber vision_info_sub_;
    vision_info_parser::VisionInfoParser parser_;
    boost::optional<std::map<int,std::string> > class_info_;
    std::map<int,std_msgs::ColorRGBA> color_lists_;
    ros::Publisher marker_pub_;
};

#endif  //VISION_MSGS_VISUALIZATION_DETECTION_3D_VISUALIZER_H_INCLUDED