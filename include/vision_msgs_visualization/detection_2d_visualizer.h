#ifndef VISION_MSGS_VISUALIZATION_DETECTION_2D_VISUALIZER_H_INCLUDED
#define VISION_MSGS_VISUALIZATION_DETECTION_2D_VISUALIZER_H_INCLUDED

// Headers in ROS
#include <ros/ros.h>
#include <vision_msgs/Detection2DArray.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

// Headers in STL
#include <memory>

class Detection2DVisualizer
{
public:
    Detection2DVisualizer(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~Detection2DVisualizer();
    const ros::NodeHandle nh;
    const ros::NodeHandle pnh;
private:
    image_transport::ImageTransport it_;
    void callback(const sensor_msgs::ImageConstPtr& image, const vision_msgs::Detection2DArrayConstPtr& detection);
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > image_sub_ptr_;
    std::shared_ptr<message_filters::Subscriber<vision_msgs::Detection2DArray> > detection_sub_ptr_;
    std::string image_topic_;
    std::string detection_topic_;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::Image, vision_msgs::Detection2DArray> > sync_ptr_;
};

#endif  //VISION_MSGS_VISUALIZATION_DETECTION_2D_VISUALIZER_H_INCLUDED