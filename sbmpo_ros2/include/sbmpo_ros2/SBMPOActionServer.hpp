#ifndef SBMPO_ROS2_ACTION_SERVER_HPP_
#define SBMPO_ROS2_ACTION_SERVER_HPP_

#include <sbmpo/SBMPO.hpp>
#include <sbmpo_msgs/action/run_sbmpo.hpp>
#include <sbmpo_ros2/SBMPOROS2Conversions.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace sbmpo_ros2 {

using namespace sbmpo;

template <typename ModelType>
class SBMPOActionServer : public rclcpp::Node {
    static_assert(std::is_base_of<sbmpo::Model, ModelType>::value, "ModelType must derive from sbmpo::Model");

public:

    using RunSBMPOAction = sbmpo_msgs::action::RunSBMPO;
    using GoalHandleRunSBMPOAction = rclcpp_action::ServerGoalHandle<RunSBMPOAction>;

    SBMPOActionServer(const std::string& planner_name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node(planner_name + "_server_node", options), running_(false), update_rate_ms_(100L) {

        // Parameters
        this->declare_parameter("feedback_update_rate_ms", int(update_rate_ms_));
        update_rate_ms_ = this->get_parameter("feedback_update_rate_ms").as_int();

        // Action server
        action_server_ = rclcpp_action::create_server<RunSBMPOAction>(
            this,
            "run_" + planner_name,
            std::bind(&SBMPOActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&SBMPOActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&SBMPOActionServer::handle_accepted, this, std::placeholders::_1));

        // Feedback timer
        feedback_timer_ = this->create_wall_timer(std::chrono::milliseconds(update_rate_ms_),
            std::bind(&SBMPOActionServer::feedback, this));

        RCLCPP_INFO(this->get_logger(), (planner_name + "_server_node initialized.").c_str());
    }

protected:

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const RunSBMPOAction::Goal> goal) {
        (void)uuid; (void)goal;
        RCLCPP_INFO(this->get_logger(), "Received goal request");
        if (running_) {
            RCLCPP_INFO(this->get_logger(), "Request rejected");
            return rclcpp_action::GoalResponse::REJECT;
        } else {
            RCLCPP_INFO(this->get_logger(), "Request accepted");
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleRunSBMPOAction> goal_handle) {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        sbmpo_.quit();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleRunSBMPOAction> goal_handle) {
        goal_handle_ = goal_handle;
        const SearchParameters params = fromParametersMsg(goal_handle_->get_goal()->parameters);
        RCLCPP_INFO(this->get_logger(), "Running planner.");
        std::thread{std::bind(&SBMPOActionServer::execute, this, std::placeholders::_1), params}.detach();
    }

    void execute(const SearchParameters& params) {
        running_ = true;
        sbmpo_.run(params);
        RCLCPP_INFO(this->get_logger(), "Planning finished.");
        auto action_results = std::make_shared<RunSBMPOAction::Result>();
        action_results->results = toResultsMsg(sbmpo_.results());
        if (sbmpo_.exit_code() == sbmpo::QUIT_SEARCH) {
            goal_handle_->canceled(action_results);
        } else {
            goal_handle_->succeed(action_results);
        }
        running_ = false;
        RCLCPP_INFO(this->get_logger(), "Results published.");
    }

    void feedback() {
        if (running_) {
            auto feedback = std::make_shared<RunSBMPOAction::Feedback>();
            feedback->time_us = sbmpo_.time_us();
            feedback->iteration = sbmpo_.iterations();
            goal_handle_->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Feedback published.");
        }
    }

    SBMPO<ModelType> sbmpo_;
    rclcpp_action::Server<RunSBMPOAction>::SharedPtr action_server_;
    std::shared_ptr<GoalHandleRunSBMPOAction> goal_handle_;
    rclcpp::TimerBase::SharedPtr feedback_timer_;
    bool running_;
    time_t update_rate_ms_;
};

}

#endif
