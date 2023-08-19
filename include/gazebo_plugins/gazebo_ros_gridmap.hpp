// Copyright 2022 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_GRIDMAP_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_GRIDMAP_HPP_

// For std::unique_ptr, could be removed
#include <memory>

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include "grid_map_ros/grid_map_ros.hpp"

namespace gazebo_plugins
{
// Forward declaration of private data class.
class GazeboRosGridmapPrivate;

class GazeboRosGridmap : public gazebo::WorldPlugin
{
public:
  /// Constructor
  GazeboRosGridmap();

  /// Destructor
  virtual ~GazeboRosGridmap();

  /// Gazebo calls this when the plugin is loaded.
  /// \param[in] model Pointer to parent model. Other plugin types will expose different entities,
  /// such as `gazebo::sensors::SensorPtr`, `gazebo::physics::WorldPtr`,
  /// `gazebo::rendering::VisualPtr`, etc.
  /// \param[in] sdf SDF element containing user-defined parameters.
  void Load(gazebo::physics::WorldPtr model, sdf::ElementPtr sdf) override;

protected:
  /// Optional callback to be called at every simulation iteration.
  virtual void OnUpdate();

  double get_surface(
    const ignition::math::Vector3d & central_point,
    const double min_z, const double max_z, const double last_valid_z,
    const double resolution, gazebo::physics::RayShapePtr ray);
  void create_gridmap();
  void create_octomap();
  bool is_obstacle(
    const ignition::math::Vector3d & central_point, double surface,
    const double min_z, const double max_z,
    const double resolution,
    gazebo::physics::RayShapePtr ray);
  bool voxel_is_obstacle(
    const ignition::math::Vector3d & central_point,
    const double resolution,
    gazebo::physics::RayShapePtr ray);

  void flood(const grid_map::Position & current_pos, double current_height,
    double resolution, gazebo::physics::RayShapePtr ray);

private:
  /// Recommended PIMPL pattern. This variable should hold all private
  /// data members.
  std::unique_ptr<GazeboRosGridmapPrivate> impl_;
};
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_GRIDMAP_HPP_
