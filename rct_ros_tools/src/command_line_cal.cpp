#include <ros/ros.h>
#include <rct_ros_tools/data_set.h>

#include <tf2_ros/transform_listener.h>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <eigen_conversions/eigen_msg.h>

#include <std_srvs/Empty.h>

#include <rct_image_tools/image_observation_finder.h>

struct DataCollection
{
  std::string base_frame;
  std::string tool_frame;

  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener;

  image_transport::ImageTransport it;
  image_transport::Subscriber im_sub;
  image_transport::Publisher im_pub;
  cv_bridge::CvImagePtr last_frame;

  ros::ServiceServer trigger_server;
  ros::ServiceServer save_server;

  std::vector<geometry_msgs::TransformStamped> poses;
  std::vector<cv_bridge::CvImagePtr> images;

  rct_image_tools::ModifiedCircleGridObservationFinder finder;

  DataCollection(ros::NodeHandle& nh)
    : listener(buffer)
    , it(nh)
    , finder (rct_image_tools::ModifiedCircleGridTarget(5, 5, 0.015))
  {
    ros::NodeHandle pnh ("~");
    pnh.getParam("base", base_frame);
    pnh.getParam("tool", tool_frame);

    im_sub = it.subscribe("camera", 1, &DataCollection::onNewImage, this);
    im_pub = it.advertise("observer", 1);

    trigger_server = nh.advertiseService("collect", &DataCollection::onTrigger, this);
    save_server = nh.advertiseService("save", &DataCollection::onSave, this);
  }

  void onNewImage(const sensor_msgs::ImageConstPtr& msg)
  {
    ROS_INFO_STREAM("New image");
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);


      auto obs = finder.findObservations(cv_ptr->image);
      if (obs)
      {
        auto modified = finder.drawObservations(cv_ptr->image, *obs);
        cv_bridge::CvImagePtr ptr (new cv_bridge::CvImage(cv_ptr->header, cv_ptr->encoding, modified));
        im_pub.publish(ptr->toImageMsg());
      }
      else
      {
        im_pub.publish(cv_ptr->toImageMsg());
      }

      last_frame = cv_ptr;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }

  bool onTrigger(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&)
  {
    geometry_msgs::TransformStamped tf = buffer.lookupTransform(base_frame, tool_frame, ros::Time(), ros::Duration(5));
    if (last_frame)
    {
      poses.push_back(tf);
      images.push_back(last_frame);
    }

    return true;
  }

  bool onSave(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&)
  {
    rct_ros_tools::ExtrinsicDataSet data;
    for (std::size_t i = 0; i < poses.size(); ++i)
    {
      cv::Mat image = images[i]->image;
      auto msg = poses[i];
      Eigen::Affine3d pose;
      tf::transformMsgToEigen(msg.transform, pose);

      data.images.push_back(image);
      data.tool_poses.push_back(pose);
    }

    rct_ros_tools::saveToDirectory("cal_test", data);
    return true;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rct_examples");
  ros::NodeHandle nh;
  DataCollection dc (nh);
  ros::spin();
  return 0;
}
