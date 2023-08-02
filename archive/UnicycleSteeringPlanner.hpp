#ifndef SBMPO_ROS2_UNICYCLE_STEERING_PLANNER_HPP
#define SBMPO_ROS2_UNICYCLE_STEERING_PLANNER_HPP

#include <sbmpo/models/UnicycleSteeringModel.hpp>
#include <sbmpo_ros2/planner_node.hpp>
#include <sbmpo_ros2/ros2_models/OccupancyGridModel.hpp>

namespace sbmpo_ros2 {

using namespace sbmpo;

using BaseModel = sbmpo_ros2::OccupancyGridModel<sbmpo_models::UnicycleSteeringModel>;

class UnicycleSteeringPlanner : public SBMPOROS2Planner<BaseModel> {

public:

    UnicycleSteeringPlanner() : SBMPOROS2Planner<BaseModel>("unicycle_steering_planner") {

        this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "odom",
            10,
            [&](nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid_msg) {
                static_cast<BaseModel*>(model_.get())->set_grid_map(grid_msg);
            }
        );

    }

    State start_state() override {

        const auto position = odometry_->pose.pose.position;
        const auto orientation = odometry_->pose.pose.orientation;

        State start(3);
        start[0] = position.x;
        start[1] = position.y;
        start[2] = quaternion_to_yaw(orientation);

        return start;
    }

    State goal_state() override {
        
        State goal(3);
        goal[0] = goal_point_->point.x;
        goal[1] = goal_point_->point.y;
        goal[2] = 0.0f;

        return goal;
    }

    nav_msgs::msg::Path to_path(const std::vector<State> &state_path, const std::vector<Control> &control_path) override {

        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = rclcpp::Clock().now();
        path_msg.header.frame_id = "map";
        
        for (const State& state : state_path) {
            geometry_msgs::msg::PoseStamped poseStamped;
            poseStamped.header.frame_id = "map";
            poseStamped.header.stamp = rclcpp::Clock().now();
            poseStamped.pose.position.x = state[0];
            poseStamped.pose.position.y = state[1];
            poseStamped.pose.orientation.x = 0;
            poseStamped.pose.orientation.y = 0;
            poseStamped.pose.orientation.z = sin(state[2] / 2);
            poseStamped.pose.orientation.w = cos(state[2] / 2);
            path_msg.poses.push_back(poseStamped);
        }
        
        return path_msg;

    }

protected:

    float quaternion_to_yaw(const geometry_msgs::msg::Quaternion& quaternion) {
        const double w = quaternion.w;
        const double x = quaternion.x;
        const double y = quaternion.y;
        const double z = quaternion.z;
        
        //const double roll = atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y));
        //const double pitch = asin(2*(w*y - z*x));
        const double yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z));
        
        return float(yaw);
    }

};

};

#endif