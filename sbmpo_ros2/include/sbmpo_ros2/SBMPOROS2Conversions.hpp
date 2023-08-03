#ifndef SBMPO_ROS2_CONVERSIONS_HPP_
#define SBMPO_ROS2_CONVERSIONS_HPP_

#include <sbmpo/types/types.hpp>
#include <sbmpo_msgs/msg/state.hpp>
#include <sbmpo_msgs/msg/control.hpp>
#include <sbmpo_msgs/msg/search_parameters.hpp>
#include <sbmpo_msgs/msg/search_results.hpp>

namespace sbmpo_ros2 {

/*
    ROS2 Message to SBMPO Conversions
*/

inline sbmpo::State fromStateMsg(const sbmpo_msgs::msg::State &msg) {
    return msg.data;
}

inline sbmpo::Control fromControlMsg(const sbmpo_msgs::msg::Control &msg) {
    return msg.data;
}

inline sbmpo::SearchParameters fromParametersMsg(const sbmpo_msgs::msg::SearchParameters& msg) {
    sbmpo::SearchParameters params;
    params.max_iterations = msg.max_iterations;
    params.max_generations = msg.max_generations;
    params.time_limit_us = msg.time_limit_us;
    params.sample_time = msg.sample_time;
    params.grid_resolution = msg.grid_resolution;
    params.start_state = fromStateMsg(msg.start_state);
    params.goal_state = fromStateMsg(msg.goal_state);
    params.samples.reserve(msg.samples.size());
    for (sbmpo_msgs::msg::Control control : msg.samples)
        params.samples.emplace_back(fromControlMsg(control));
    return params;
}

inline sbmpo::SearchResults fromResultsMsg(const sbmpo_msgs::msg::SearchResults& msg) {
    sbmpo::SearchResults results;
    results.time_us = msg.time_us;
    results.exit_code = (sbmpo::ExitCode) msg.exit_code;
    results.iteration = msg.iterations;
    results.node_count = msg.node_count;
    results.cost = msg.cost;
    results.state_path.reserve(msg.state_path.size());
    for (sbmpo_msgs::msg::State state : msg.state_path)
        results.state_path.emplace_back(fromStateMsg(state));
    results.control_path.reserve(msg.control_path.size());
    for (sbmpo_msgs::msg::Control control : msg.control_path)
        results.control_path.emplace_back(fromControlMsg(control));
    return results;
}

/*
    SBMPO to ROS2 Message Conversions
*/

inline sbmpo_msgs::msg::State toStateMsg(const sbmpo::State& state) {
    sbmpo_msgs::msg::State msg;
    msg.data = state;
    return msg;
}

inline sbmpo_msgs::msg::Control toControlMsg(const sbmpo::Control& control) {
    sbmpo_msgs::msg::Control msg;
    msg.data = control;
    return msg;
}

inline sbmpo_msgs::msg::SearchParameters toParametersMsg(const sbmpo::SearchParameters& params) {
    sbmpo_msgs::msg::SearchParameters msg;
    msg.max_iterations = params.max_iterations;
    msg.max_generations = params.max_generations;
    msg.sample_time = params.sample_time;
    msg.grid_resolution = params.grid_resolution;
    msg.start_state = toStateMsg(params.start_state);
    msg.goal_state = toStateMsg(params.goal_state);
    msg.samples.reserve(params.samples.size());
    for (sbmpo::Control control : params.samples)
        msg.samples.emplace_back(toControlMsg(control));
    return msg;
}

inline sbmpo_msgs::msg::SearchResults toResultsMsg(const sbmpo::SearchResults& results) {
    sbmpo_msgs::msg::SearchResults msg;
    msg.time_us = results.time_us;
    msg.exit_code = (sbmpo::ExitCode) results.exit_code;
    msg.iterations = results.iteration;
    msg.node_count = results.node_count;
    msg.cost = results.cost;
    msg.state_path.reserve(results.state_path.size());
    for (sbmpo::State state : results.state_path)
        msg.state_path.emplace_back(toStateMsg(state));
    msg.control_path.reserve(results.control_path.size());
    for (sbmpo::Control control : results.control_path)
        msg.control_path.emplace_back(toControlMsg(control));
    return msg;
}

}

#endif