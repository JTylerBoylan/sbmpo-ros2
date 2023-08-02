#include <rclcpp/rclcpp.hpp>
#include <sbmpo_ros2/SBMPOActionClient.hpp>
#include <sbmpo/models/UnicycleSteeringModel.hpp>

#include <rclcpp_components/register_node_macro.hpp>

namespace sbmpo_ros2 {

using namespace sbmpo;

class UnicycleSteeringActionClient : public SBMPOActionClient {

public:

    UnicycleSteeringActionClient(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : SBMPOActionClient("unicycle_steering", options) {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2000),
            [&]() {
                if (!this->is_running())
                    this->run_planner(get_parameters());
            }
        );
    }

    SearchParameters get_parameters() {
        SearchParameters parameters;
        parameters.start_state = {0, 0, 0};
        parameters.goal_state = {10, 0, 0};
        parameters.grid_resolution = {0.05, 0.05, -1};
        parameters.max_iterations = 5000;
        parameters.max_generations = 50;
        parameters.sample_time = 1.0;
        parameters.samples = {
            {0.25, -M_2PI/8.0},
            {0.25, 0.0},
            {0.25, M_2PI/8.0}
        };
        return parameters;
    }

private:

    rclcpp::TimerBase::SharedPtr timer_;

};

}

RCLCPP_COMPONENTS_REGISTER_NODE(sbmpo_ros2::UnicycleSteeringActionClient)