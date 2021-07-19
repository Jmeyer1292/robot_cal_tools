#include <rct_ros_tools/target_loaders.h>

namespace rct_ros_tools
{
class ModifiedCircleGridTargetFinderPlugin : public TargetFinderPlugin
{
public:
  using TargetFinderPlugin::TargetFinderPlugin;

  void init(const XmlRpc::XmlRpcValue& config) override;
  void init(const std::string& file) override;
};

class CharucoGridTargetFinderPlugin : public TargetFinderPlugin
{
public:
  using TargetFinderPlugin::TargetFinderPlugin;

  void init(const XmlRpc::XmlRpcValue& config) override;
  void init(const std::string& file) override;
};

}
