#include <rclcpp/rclcpp.hpp>
#include <sbmpo_ros2/SBMPOActionServer.hpp>
#include <sbmpo/models/UnicycleSteeringModel.hpp>

#include <rclcpp_components/register_node_macro.hpp>

namespace sbmpo_ros2 {

using namespace sbmpo_models;

class UnicycleSteeringActionServer : public SBMPOActionServer<UnicycleSteeringModel> {

public:

    UnicycleSteeringActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : SBMPOActionServer<UnicycleSteeringModel>("unicycle_steering", options) {}

};

}

RCLCPP_COMPONENTS_REGISTER_NODE(sbmpo_ros2::UnicycleSteeringActionServer)