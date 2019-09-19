// headers in this package
#include <vision_msgs_visualization/detection_3d_visualizer.h>

Detection3DVisualizer::Detection3DVisualizer(ros::NodeHandle nh,ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    pnh.param<std::string>("detection_topic", detection_topic_, "detection_2d");
    pnh.param<std::string>("vision_info_topic", vision_info_topic_, "vision_info");
    pnh.param<bool>("publish_model_marker", publish_model_marker_, false);
    marker_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("marker",1);
    detection_sub_ = nh_.subscribe(detection_topic_,1,&Detection3DVisualizer::detectionCallback,this);
    vision_info_sub_ = nh_.subscribe(vision_info_topic_,1,&Detection3DVisualizer::visionInfoCallback,this);
}

Detection3DVisualizer::~Detection3DVisualizer()
{

}

void Detection3DVisualizer::detectionCallback(const vision_msgs::Detection3DArray::ConstPtr msg)
{
    if(class_info_)
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
        visualization_msgs::Marker object_marker;
        object_marker.header = itr->header;
        object_marker.id = i;
        object_marker.action = visualization_msgs::Marker::ADD;
        object_marker.pose = itr->bbox.center;
        std::string class_str;
        if(publish_model_marker_)
        {
            object_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            object_marker.ns = "model";
            object_marker.mesh_use_embedded_materials = true;
            object_marker.scale.x = 1.0;
            object_marker.scale.y = 1.0;
            object_marker.scale.z = 1.0;
            if(itr->results.size() > 0)
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
                class_str = class_info_.get()[itr->results[index].id];
                object_marker.color = color_lists_[itr->results[index].id];
            }
            else
            {
                object_marker.color = color_lists_[0];
            }
            if(class_str != "")
            {
                object_marker.mesh_resource = model_path_lists_[class_str];
            }
        }
        else
        {
            object_marker.type = visualization_msgs::Marker::CUBE;
            object_marker.ns = "bbox";
            object_marker.scale = itr->bbox.size;
            if(itr->results.size() > 0)
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
                class_str = class_info_.get()[itr->results[index].id];
                object_marker.color = color_lists_[itr->results[index].id];
            }
            else
            {
                object_marker.color = color_lists_[0];
            }
        }
        object_marker.lifetime = ros::Duration(0.01);
        object_marker.frame_locked = true;
        ret.markers.push_back(object_marker);
        visualization_msgs::Marker text_marker;
        text_marker.header = itr->header;
        text_marker.ns = "text";
        text_marker.id = i;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        if(itr->results.size() > 0)
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
            text_marker.color = color_lists_[itr->results[index].id];
            text_marker.color.a = 1.0;
            text_marker.text = "class:"+class_info_.get()[itr->results[index].id]+"\nscore:"+std::to_string(itr->results[index].score*100)+"%";
        }
        else
        {
            text_marker.color = color_lists_[0];
            text_marker.color.a = 1.0;
            text_marker.text = "object";
        }
        text_marker.pose = itr->bbox.center;
        text_marker.pose.position.z = text_marker.pose.position.z + itr->bbox.size.z*0.5 + 0.8;
        text_marker.scale.x = 0.4;
        text_marker.scale.y = 0.4;
        text_marker.scale.z = 0.4;
        text_marker.lifetime = ros::Duration(0.01);
        text_marker.frame_locked = true;
        ret.markers.push_back(text_marker);
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
        model_path_lists_.clear();
        for(auto itr=class_info_->begin(); itr!=class_info_->end(); itr++)
        {
            std::string path;
            pnh_.param<std::string>("model_path/"+itr->second, path, "");
            bool override_flag = false;
            pnh_.param<bool>(itr->second+"/color/override", override_flag, false);
            if(override_flag)
            {
                std_msgs::ColorRGBA color;
                double r,g,b,a;
                pnh_.param<double>(itr->second+"/color/r", r, 1.0);
                pnh_.param<double>(itr->second+"/color/g", g, 1.0);
                pnh_.param<double>(itr->second+"/color/b", b, 1.0);
                pnh_.param<double>(itr->second+"/color/a", a, 1.0);
                color.r = r;
                color.g = g;
                color.b = b;
                color.a = a;
                color_lists_[itr->first] = color;
            }
            else
            {
                std_msgs::ColorRGBA color;
                color.r = random(mt);
                color.g = random(mt);
                color.b = random(mt);
                color.a = 0.8;
                color_lists_[itr->first] = color;
            }
            model_path_lists_[itr->second] = path;
        }
    }
    return;
}