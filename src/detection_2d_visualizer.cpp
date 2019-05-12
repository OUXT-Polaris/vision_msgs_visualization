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
    return;
}

void Detection2DVisualizer::callback(const sensor_msgs::ImageConstPtr& image, const vision_msgs::Detection2DArrayConstPtr& detection)
{
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
    return;
}