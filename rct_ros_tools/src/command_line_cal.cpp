#include <ros/ros.h>
#include <rct_ros_tools/data_set.h>
#include <rct_ros_tools/parameter_loaders.h>
#include <rct_ros_tools/target_loaders.h>

#include <tf2_ros/transform_listener.h>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <eigen_conversions/eigen_msg.h>

#include <std_srvs/Empty.h>

#include <rct_image_tools/modified_circle_grid_finder.h>

class TransformMonitor
{
public:
  TransformMonitor(const std::string& base_frame, const std::string& tool_frame)
    : base_frame_(base_frame)
    , tool_frame_(tool_frame)
    , listener_(buffer_)
  {
    // Validate that we can look up required transforms
    geometry_msgs::TransformStamped dummy;
    if (!capture(dummy))
    {
      throw std::runtime_error("Transform from " + base_frame_ + " to " + tool_frame_ + " not available");
    }
  }

  bool capture(geometry_msgs::TransformStamped& out)
  {
    try
    {
      geometry_msgs::TransformStamped t = buffer_.lookupTransform(base_frame_, tool_frame_, ros::Time(), ros::Duration(3.0));
      out = t;
      return true;
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_STREAM("Failed to compute transfrom between " << base_frame_ << " and " << tool_frame_ << ": "
                      << ex.what());
      return false;
    }
  }

private:
  std::string base_frame_;
  std::string tool_frame_;

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
};

class ImageMonitor
{
public:
  ImageMonitor(const rct_image_tools::ModifiedCircleGridTargetFinder& finder,
               const std::string& nominal_image_topic)
    : finder_(finder)
    , it_(ros::NodeHandle())
  {
    im_sub_ = it_.subscribe(nominal_image_topic, 1, &ImageMonitor::onNewImage, this);
    im_pub_ = it_.advertise(nominal_image_topic + "_observer", 1);
  }

  void onNewImage(const sensor_msgs::ImageConstPtr& msg)
  {
    ROS_INFO_STREAM("New image");
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      if(msg->encoding == "mono16")
      {
        cv_bridge::CvImagePtr temp_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);

        cv::Mat img_conv;
        cv::cvtColor(temp_ptr->image, img_conv, cv::COLOR_GRAY2BGR);
        img_conv.convertTo(img_conv, CV_8UC1);
        cv_ptr = cv_bridge::CvImagePtr(new cv_bridge::CvImage(temp_ptr->header, sensor_msgs::image_encodings::BGR8, img_conv));
      }
      else
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }

      rct_image_tools::TargetFeatures image_observations;
      try
      {
        image_observations = finder_.findTargetFeatures(cv_ptr->image);

        auto modified = finder_.drawTargetFeatures(cv_ptr->image, image_observations);
        cv_bridge::CvImagePtr ptr(new cv_bridge::CvImage(cv_ptr->header, cv_ptr->encoding, modified));
        im_pub_.publish(ptr->toImageMsg());
      }
      catch (const std::runtime_error& ex)
      {
        ROS_ERROR_STREAM(ex.what());
        im_pub_.publish(cv_ptr->toImageMsg());
      }

      last_frame_ = cv_ptr;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }

  bool capture(cv::Mat& frame)
  {
    if (last_frame_)
    {
      frame = last_frame_->image;
      return true;
    }
    return false;
  }

private:
  rct_image_tools::ModifiedCircleGridTargetFinder finder_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber im_sub_;
  image_transport::Publisher im_pub_;
  cv_bridge::CvImagePtr last_frame_;
};

struct DataCollectionConfig
{
  std::string base_frame;
  std::string tool_frame;

  std::string image_topic;
  std::shared_ptr<rct_image_tools::ModifiedCircleGridTarget> target;

  std::string save_dir;
};

struct DataCollection
{

  DataCollection(const DataCollectionConfig& config)
    : tf_monitor(config.base_frame, config.tool_frame)
    , image_monitor(*config.target, config.image_topic)
    , save_dir_(config.save_dir)
  {
    ros::NodeHandle nh;
    trigger_server = nh.advertiseService("collect", &DataCollection::onTrigger, this);
    save_server = nh.advertiseService("save", &DataCollection::onSave, this);

    ROS_INFO_STREAM("Call " << trigger_server.getService() << " to capture a pose/image pair");
    ROS_INFO_STREAM("Call " << save_server.getService() << " to save the captured data");
  }

  bool onTrigger(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&)
  {
    ROS_INFO_STREAM("Pose/Image capture triggered...");
    geometry_msgs::TransformStamped pose;
    cv::Mat image;

    if (tf_monitor.capture(pose) && image_monitor.capture(image))
    {
      poses.push_back(pose);
      images.push_back(image);
      ROS_INFO_STREAM("Data collected successfully");
      return true;
    }
    else
    {
      ROS_WARN_STREAM("Failed to capture pose/image pair");
      return false;
    }
  }

  bool onSave(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&)
  {
    rct_ros_tools::ExtrinsicDataSet data;
    for (std::size_t i = 0; i < poses.size(); ++i)
    {
      cv::Mat image = images[i];
      auto msg = poses[i];
      Eigen::Isometry3d pose;
      tf::transformMsgToEigen(msg.transform, pose);

      data.images.push_back(image);
      data.tool_poses.push_back(pose);
    }

    ROS_INFO_STREAM("Saving data-set to " << save_dir_);
    rct_ros_tools::saveToDirectory(save_dir_, data);
    return true;
  }

  ros::ServiceServer trigger_server;
  ros::ServiceServer save_server;

  std::vector<geometry_msgs::TransformStamped> poses;
  std::vector<cv::Mat> images;

  TransformMonitor tf_monitor;
  ImageMonitor image_monitor;

  std::string save_dir_;
};

template <typename T>
bool get(const ros::NodeHandle& nh, const std::string& key, T& value)
{
  if (!nh.getParam(key, value))
  {
    ROS_ERROR_STREAM("Must set parameter, " << key << "!");
    return false;
  }
  ROS_INFO_STREAM("Parameter " << key << " set to: " << value);
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rct_examples");
  ros::NodeHandle pnh ("~");

  // Load data collection parameters
  DataCollectionConfig config;

  if (!get(pnh, "base_frame", config.base_frame)) return 1;
  if (!get(pnh, "tool_frame", config.tool_frame)) return 1;
  if (!get(pnh, "image_topic", config.image_topic)) return 1;
  if (!get(pnh, "save_dir", config.save_dir))
    return 1;

  using namespace rct_ros_tools;
  using namespace rct_image_tools;

  try
  {
    config.target = std::make_shared<ModifiedCircleGridTarget>(TargetLoader<ModifiedCircleGridTarget>::load(pnh, "target_definition"));
    DataCollection dc(config);
    ros::spin();
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return -1;
  }

  return 0;
}
