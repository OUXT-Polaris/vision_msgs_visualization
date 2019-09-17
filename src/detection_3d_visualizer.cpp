// headers in this package
#include <vision_msgs_visualization/detection_3d_visualizer.h>

Detection3DVisualizer::Detection3DVisualizer(ros::NodeHandle nh,ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    pnh.param<std::string>("detection_topic", detection_topic_, "detection_2d");
    pnh.param<std::string>("vision_info_topic", vision_info_topic_, "vision_info");
    marker_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("marker",1);
    detection_sub_ = nh_.subscribe(detection_topic_,1,&Detection3DVisualizer::detectionCallback,this);
    vision_info_sub_ = nh_.subscribe(vision_info_topic_,1,&Detection3DVisualizer::visionInfoCallback,this);
}

Detection3DVisualizer::~Detection3DVisualizer()
{

}

void Detection3DVisualizer::detectionCallback(const vision_msgs::Detection3DArray::ConstPtr msg)
{

}

void Detection3DVisualizer::visionInfoCallback(const vision_msgs::VisionInfo::ConstPtr msg)
{
    parser_.parseFromRosMessage(*msg);
    class_info_ = parser_.getClasses();
    return;
}