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
    if(!class_info_)
    {
        boost::optional<visualization_msgs::MarkerArray> marker_msg = generateMarker(*msg);
        if(marker_msg)
        {
            marker_pub_.publish(*marker_msg);
        }
        return;
    }
    return;
}

boost::optional<visualization_msgs::MarkerArray> Detection3DVisualizer::generateMarker(vision_msgs::Detection3DArray detection)
{
    visualization_msgs::MarkerArray ret;
    int i=0;
    for(auto itr=detection.detections.begin(); itr!=detection.detections.end(); itr++)
    {
        if(itr->results.size()==0)
        {
            return boost::none;
        }
        visualization_msgs::Marker cube_marker;
        cube_marker.header = itr->header;
        cube_marker.ns = "bbox";
        cube_marker.id = i;
        cube_marker.type = visualization_msgs::Marker::CUBE;
        cube_marker.action = visualization_msgs::Marker::ADD;
        cube_marker.pose = itr->bbox.center;
        cube_marker.scale = itr->bbox.size;
        if(itr->results.size() > 1)
        {
            int index = 0;
            double score = itr->results[0].score;
            for(int i=0; i<itr->results.size(); i++)
            {
                if(score<itr->results[i].score)
                {
                    index = i;
                }
            }
            cube_marker.color = color_lists_[itr->results[index].id];
        }
        else
        {
            cube_marker.color = color_lists_[0];
        }
        cube_marker.lifetime = ros::Duration(0.1);
        cube_marker.frame_locked = true;
        i++;
    }
    return ret;
}

void Detection3DVisualizer::visionInfoCallback(const vision_msgs::VisionInfo::ConstPtr msg)
{
    parser_.parseFromRosMessage(*msg);
    class_info_ = parser_.getClasses();
    if(class_info_)
    {
        std::random_device rnd;
        std::mt19937 mt(rnd());
        std::uniform_real_distribution<> random(0.0, 1.0);
        color_lists_ = std::map<int,std_msgs::ColorRGBA>();
        for(auto itr=class_info_->begin(); itr!=class_info_->end(); itr++)
        {
            std_msgs::ColorRGBA color;
            color.r = random(mt);
            color.g = random(mt);
            color.b = random(mt);
            color.a = 0.8;
            color_lists_[itr->first] = color;
        }
    }
    return;
}