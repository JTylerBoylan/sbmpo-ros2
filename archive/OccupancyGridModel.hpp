#ifndef SBMPO_ROS2_UNICYCLE_STEERING_MODEL_HPP
#define SBMPO_ROS2_UNICYCLE_STEERING_MODEL_HPP

#include <sbmpo/types/types.hpp>
#include <sbmpo/types/Model.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace sbmpo_ros2 {

using namespace sbmpo;

template<typename ModelType>
class OccupancyGridModel : public ModelType {
static_assert(std::is_base_of<sbmpo::Model, ModelType>::value, "ModelType must derive from sbmpo::Model");

    public:

    OccupancyGridModel() {}

    bool is_valid(const State& state) override {
        return ModelType::is_valid(state) && !is_occupied(state[0], state[1]);
    }

    void set_grid_map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr occupancy_grid) {
        occupancy_grid_ = occupancy_grid;
    }

    protected:

    nav_msgs::msg::OccupancyGrid::ConstSharedPtr occupancy_grid_;

    bool is_occupied(const int x, const int y) {
        const int width = occupancy_grid_->info.width;
        const int height = occupancy_grid_->info.height;
        const float resolution = occupancy_grid_->info.resolution;
        int x_cell = (int)std::round((x - occupancy_grid_->info.origin.position.x) / resolution);
        int y_cell = (int)std::round((y - occupancy_grid_->info.origin.position.y) / resolution);
        
        if (x_cell < 0 || x_cell >= width || y_cell < 0 || y_cell >= height) {
            return false;
        }
        
        const int index = y_cell * width + x_cell;
        const float occ_prob = static_cast<float>(occupancy_grid_->data[index]) / 100.0f;

        return occ_prob > 0.5f;
    }


};

};

#endif