#include <rct_ros_tools/target_finder_plugin.h>
#include <rct_ros_tools/loader_utils.h>

#include <gtest/gtest.h>
#include <pluginlib/class_loader.h>
#include <regex>
#include <ros/ros.h>

const static std::string TARGET_FILENAME_PARAM = "target_file";
const static std::string TARGET_DEFINITION_PARAM = "target_finder";

TEST(Tools, TargetFinderPluginTest)
{
  ros::NodeHandle pnh("~");

  // Create the plugin loader
  const std::string package = "rct_ros_tools";
  const std::string base_class = "rct_ros_tools::TargetFinderPlugin";
  pluginlib::ClassLoader<rct_ros_tools::TargetFinderPlugin> loader(package, base_class);

  // Get the declared plugins known to this base class
  std::vector<std::string> plugins = loader.getDeclaredClasses();

  // Check the number of defined plugins
  ASSERT_EQ(plugins.size(), 2);

  // Attempt to load and initialize each plugin using the two available methods
  for (const std::string plugin : plugins)
  {
    // Extract a namespace from the second part of the plugin name
    std::regex re(".*::(.*)");
    std::smatch match;
    ASSERT_TRUE(std::regex_search(plugin, match, re));
    const std::string ns = match[1];

    // Create the plugin
    boost::shared_ptr<rct_ros_tools::TargetFinderPlugin> finder;
    ASSERT_NO_THROW(finder = loader.createInstance(plugin));

    // Initialize the plugin
    const std::string target_definition_param = ns + "/" + TARGET_DEFINITION_PARAM;
    XmlRpc::XmlRpcValue config;
    ASSERT_TRUE(pnh.getParam(target_definition_param, config));
    ASSERT_NO_THROW(finder->init(rct_ros_tools::toYAML(config)));
  }
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "target_loader_utest");
  return RUN_ALL_TESTS();
}
