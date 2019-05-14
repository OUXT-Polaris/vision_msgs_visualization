// headers in this package
#include <vision_msgs_visualization/detection_2d_visualizer.h>

Detection2DVisualizer::Detection2DVisualizer(ros::NodeHandle nh,ros::NodeHandle pnh) : nh(nh),pnh(pnh), it_(pnh)
{
    pnh.param<std::string>("image_topic", image_topic_, "image_raw");
    pnh.param<std::string>("detection_topic", detection_topic_, "detection_2d");
    pnh.param<std::string>("vision_info_topic", vision_info_topic_, "vision_info");
    image_pub_ = it_.advertise(image_topic_+"/annotated", 1);
    image_sub_ptr_ = std::make_shared<message_filters::Subscriber<sensor_msgs::Image> >(nh,image_topic_, 1);
    detection_sub_ptr_ = std::make_shared<message_filters::Subscriber<vision_msgs::Detection2DArray> >(nh,detection_topic_, 1);
    sync_ptr_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::Image, vision_msgs::Detection2DArray> >(*image_sub_ptr_,*detection_sub_ptr_,10);
    sync_ptr_->registerCallback(boost::bind(&Detection2DVisualizer::callback, this, _1, _2));
    vision_info_sub_ = nh.subscribe(vision_info_topic_,1,&Detection2DVisualizer::visionInfoCallback,this);
}

Detection2DVisualizer::~Detection2DVisualizer()
{

}

void Detection2DVisualizer::visionInfoCallback(const vision_msgs::VisionInfo::ConstPtr msg)
{
    std::string class_meta_str;
    nh.param<std::string>(msg->database_location, class_meta_str, "");
    std::stringstream ss;
    ss << class_meta_str;
    using namespace boost::property_tree;
    ptree pt;
    std::map<int,std::string> classes;
    std::map<int,cv::Scalar> colors;
    cv::RNG rng(313);
    read_xml(ss, pt);
    BOOST_FOREACH (const ptree::value_type& child, pt.get_child("vision_info"))
    {
        if(child.first == "class")
        {
            boost::optional<int> id = child.second.get_optional<int>("<xmlattr>.id");
            boost::optional<std::string> name = child.second.get_optional<std::string>("<xmlattr>.name");
            if(id && name)
            {
                cv::Scalar color = cv::Scalar(rng(256), rng(256), rng(256));
                classes[*id] = *name;
                colors[*id] = color;
            }
            else
            {
                ROS_ERROR_STREAM("failed to read xml string");
                std::exit(-1);
            }
        }
    }
    classes_ = classes;
    colors_ = colors;
    return;
}

void Detection2DVisualizer::callback(const sensor_msgs::ImageConstPtr& image, const vision_msgs::Detection2DArrayConstPtr& detection)
{
    if(!classes_)
    {
        return;
    }
    if(!colors_)
    {
        return;
    }
    std::map<int,std::string> classes = *classes_;
    std::map<int,cv::Scalar> colors = *colors_;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    for(auto detection_itr = detection->detections.begin(); detection_itr != detection->detections.end(); detection_itr++)
    {
        boost::optional<vision_msgs::ObjectHypothesisWithPose> result;
        for(auto result_itr = detection_itr->results.begin(); result_itr != detection_itr->results.end(); result_itr++)
        {
            if(!result)
            {
                result = *result_itr;
            }
            else
            {
                if(result_itr->score > result->score)
                {
                    result = *result_itr;
                }
            }
        }
        if(result)
        {
            int x0 = (int)(detection_itr->bbox.center.x-detection_itr->bbox.size_x/2.0);
            int x1 = (int)(detection_itr->bbox.center.x+detection_itr->bbox.size_x/2.0);
            int y0 = (int)(detection_itr->bbox.center.y-detection_itr->bbox.size_y/2.0);
            int y1 = (int)(detection_itr->bbox.center.y+detection_itr->bbox.size_y/2.0);
            cv::rectangle(cv_ptr->image, {x0, y0}, {x1, y1}, colors[result->id], 2);
            int text_y0 = y0 + ((y0 > 30) ? -15 : 15);
            std::string label = classes[result->id] + ":" + std::to_string(result->score*100.0) + "%";
            cv::putText(cv_ptr->image, label, {x0, text_y0}, cv::FONT_HERSHEY_SIMPLEX, 0.5, colors[result->id], 1, false);
        }
    }
    image_pub_.publish(cv_ptr->toImageMsg());
    return;
}