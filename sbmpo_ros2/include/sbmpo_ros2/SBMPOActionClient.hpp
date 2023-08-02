#ifndef SBMPO_ROS2_ACTION_CLIENT_HPP_
#define SBMPO_ROS2_ACTION_CLIENT_HPP_

#include <sbmpo/types/types.hpp>
#include <sbmpo_msgs/action/run_sbmpo.hpp>
#include <sbmpo_ros2/SBMPOROS2Conversions.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace sbmpo_ros2 {

class SBMPOActionClient : public rclcpp::Node {

public:

    using RunSBMPOAction = sbmpo_msgs::action::RunSBMPO;
    using GoalHandleRunSBMPOAction = rclcpp_action::ClientGoalHandle<RunSBMPOAction>;

    SBMPOActionClient(const std::string& planner_name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) 
    : Node(planner_name + "_client_node", options),
      client_ptr_(rclcpp_action::create_client<RunSBMPOAction>(this, "run_" + planner_name)),
      running_(false) {}

    void run_planner(const sbmpo::SearchParameters& params) {

        // Wait for action server to start
        if (!this->client_ptr_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
        }

        // Create goal message
        auto param_msg = toParametersMsg(params);
        auto goal_msg = RunSBMPOAction::Goal();
        goal_msg.parameters = param_msg;

        // Send goal
        using namespace std::placeholders;
        auto options = rclcpp_action::Client<RunSBMPOAction>::SendGoalOptions();
        options.goal_response_callback = std::bind(&SBMPOActionClient::planner_response, this, _1);
        options.feedback_callback = std::bind(&SBMPOActionClient::planner_feedback, this, _1, _2);
        options.result_callback = std::bind(&SBMPOActionClient::planner_result, this, _1);
        this->client_ptr_->async_send_goal(goal_msg, options);

        running_ = true;
        RCLCPP_INFO(this->get_logger(), "Goal sent to server.");
    }

    bool is_running() { return running_; }

protected:

    virtual void planner_response(GoalHandleRunSBMPOAction::SharedPtr goal_handle) {
        if (!goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Goal was rejected by server.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result.");
        }
    }

    virtual void planner_feedback(GoalHandleRunSBMPOAction::SharedPtr, 
        const std::shared_ptr<const RunSBMPOAction::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(), "Planner Feedback: Time: %lums, Iteration: %lu",
             feedback->time_us / 1000L, feedback->iteration);
    }

    virtual void planner_result(const GoalHandleRunSBMPOAction::WrappedResult& result) {
        running_ = false;
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted.");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled.");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
                return;
        }
        results_ = sbmpo_ros2::fromResultsMsg(result.result->results);
        RCLCPP_INFO(this->get_logger(), "Planner Result: Time: %lums, Exit code: %d, Iteration: %d, Cost: %.3f",
            results_.time_us, results_.exit_code, results_.iteration, results_.cost);
    }

    rclcpp_action::Client<RunSBMPOAction>::SharedPtr client_ptr_;
    sbmpo::SearchResults results_;
    bool running_;

};

}

#endif