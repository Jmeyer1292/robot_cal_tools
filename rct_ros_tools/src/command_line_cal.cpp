#include <rct_ros_tools/data_set.h>
#include <rct_ros_tools/parameter_loaders.h>
#include <rct_ros_tools/target_finder_plugin.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

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
  ImageMonitor(boost::shared_ptr<const rct_image_tools::TargetFinder> finder,
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
      last_frame_ = cv_ptr;

      try
      {
        cv::Mat tmp_image = cv_ptr->image.clone();
        rct_image_tools::TargetFeatures image_observations = finder_->findTargetFeatures(tmp_image);
        cv::Mat modified_image = finder_->drawTargetFeatures(tmp_image, image_observations);
        cv_bridge::CvImagePtr ptr(new cv_bridge::CvImage(cv_ptr->header, cv_ptr->encoding, modified_image));
        im_pub_.publish(ptr->toImageMsg());
      }
      catch (const std::runtime_error& ex)
      {
        ROS_ERROR_STREAM(ex.what());
        im_pub_.publish(cv_ptr->toImageMsg());
      }
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
  boost::shared_ptr<const rct_image_tools::TargetFinder> finder_;
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
  boost::shared_ptr<rct_ros_tools::TargetFinderPlugin> target_finder;

  std::string save_dir;
};

struct DataCollection
{

  DataCollection(const DataCollectionConfig& config)
    : tf_monitor(config.base_frame, config.tool_frame)
    , image_monitor(config.target_finder, config.image_topic)
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
      Eigen::Isometry3d pose = tf2::transformToEigen(msg.transform);

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
T get(const ros::NodeHandle& nh, const std::string& key)
{
  T value;
  if (!nh.getParam(key, value))
      throw std::runtime_error("Must set parameter, " + key + "!");
  ROS_INFO_STREAM("Parameter " << key << " set to: " << value);
  return value;
}

int main(int argc, char** argv)
{
  try
  {
    ros::init(argc, argv, "rct_examples");
    ros::NodeHandle pnh("~");

    // Load data collection parameters
    DataCollectionConfig config;
    config.base_frame = get<std::string>(pnh, "base_frame");
    config.tool_frame = get<std::string>(pnh, "tool_frame");
    config.image_topic = get<std::string>(pnh, "image_topic");
    config.save_dir = get<std::string>(pnh, "save_dir");
    auto target_finder_config = get<XmlRpc::XmlRpcValue>(pnh, "target_finder");
    const std::string target_finder_type = static_cast<std::string>(target_finder_config["type"]);

    using namespace rct_ros_tools;
    using namespace rct_image_tools;

    pluginlib::ClassLoader<TargetFinderPlugin> loader("rct_ros_tools", "rct_ros_tools::TargetFinderPlugin");
    config.target_finder = loader.createInstance(target_finder_type);
    config.target_finder->init(target_finder_config);

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
