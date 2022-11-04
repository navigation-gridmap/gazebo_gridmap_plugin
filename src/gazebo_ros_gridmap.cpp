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


#include <gazebo/physics/Model.hh>
#include <gazebo_plugins/gazebo_ros_gridmap.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>

#include <octomap_msgs/msg/octomap.hpp>
#include "octomap_msgs/conversions.h"

#include "octomap_ros/conversions.hpp"

#include <memory>
#include <string>
#include <utility>


#include "grid_map_ros/grid_map_ros.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"

namespace gazebo_plugins
{
/// Class to hold private data members (PIMPL pattern)
class GazeboRosGridmapPrivate
{
public:
  GazeboRosGridmapPrivate()
  : gridmap_({"elevation", "occupancy"}) {}

  /// Connection to world update event. Callback is called while this is alive.
  gazebo::event::ConnectionPtr update_connection_;

  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  gazebo::physics::WorldPtr world_;
  bool gridmap_created_{false};
  grid_map::GridMap gridmap_;
  bool octomap_created_{false};
  std::unique_ptr<octomap::OcTree> octomap_;

  double center_x_;
  double center_y_;
  double center_z_;
  double min_scan_x_;
  double min_scan_y_;
  double min_scan_z_;
  double max_scan_x_;
  double max_scan_y_;
  double max_scan_z_;
  double min_height_;
  double max_height_;
  double resolution_;

  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gridmap_pub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
};

GazeboRosGridmap::GazeboRosGridmap()
: impl_(std::make_unique<GazeboRosGridmapPrivate>())
{
}

GazeboRosGridmap::~GazeboRosGridmap()
{
}

void GazeboRosGridmap::Load(gazebo::physics::WorldPtr _parent, sdf::ElementPtr sdf)
{
  // Create a GazeboRos node instead of a common ROS node.
  // Pass it SDF parameters so common options like namespace and remapping
  // can be handled.
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

  // The model pointer gives you direct access to the physics object,
  // for example:
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Loaded %s", _parent->Name().c_str());

  // Create a connection so the OnUpdate function is called at every simulation
  // iteration. Remove this call, the connection and the callback if not needed.
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosGridmap::OnUpdate, this));

  impl_->gridmap_pub_ = impl_->ros_node_->create_publisher<grid_map_msgs::msg::GridMap>(
    "grid_map", rclcpp::QoS(1).transient_local());

  impl_->octomap_pub_ = impl_->ros_node_->create_publisher<octomap_msgs::msg::Octomap>(
    "octomap", rclcpp::QoS(1).transient_local());

  auto logger = impl_->ros_node_->get_logger();

  if (!sdf->HasElement("center_x")) {
    RCLCPP_INFO(
      logger, "Gidmap plugin missing <center_x> wasn't set,"
      "therefore it's been set as '0.0'.");
    impl_->center_x_ = 0.0;
  } else {
    impl_->center_x_ = sdf->GetElement("center_x")->Get<double>();
  }

  if (!sdf->HasElement("center_y")) {
    RCLCPP_INFO(
      logger, "Gidmap plugin missing <center_y> wasn't set,"
      "therefore it's been set as '0.0'.");
    impl_->center_y_ = 0.0;
  } else {
    impl_->center_y_ = sdf->GetElement("center_y")->Get<double>();
  }
  if (!sdf->HasElement("center_z")) {
    RCLCPP_INFO(
      logger, "Gidmap plugin missing <center_z> wasn't set,"
      "therefore it's been set as '0.0'.");
    impl_->center_z_ = 0.0;
  } else {
    impl_->center_z_ = sdf->GetElement("center_z")->Get<double>();
  }

  if (!sdf->HasElement("min_scan_x")) {
    RCLCPP_INFO(
      logger, "Gidmap plugin missing <min_scan_x> wasn't set,"
      "therefore it's been set as '-5.0'.");
    impl_->min_scan_x_ = -5.0;
  } else {
    impl_->min_scan_x_ = sdf->GetElement("min_scan_x")->Get<double>();
  }

  if (!sdf->HasElement("min_scan_y")) {
    RCLCPP_INFO(
      logger, "Gidmap plugin missing <min_scan_y> wasn't set,"
      "therefore it's been set as '-5.0'.");
    impl_->min_scan_y_ = -5.0;
  } else {
    impl_->min_scan_y_ = sdf->GetElement("min_scan_y")->Get<double>();
  }

  if (!sdf->HasElement("min_scan_z")) {
    RCLCPP_INFO(
      logger, "Gidmap plugin missing <min_scan_z> wasn't set,"
      "therefore it's been set as '-5.0'.");
    impl_->min_scan_z_ = -5.0;
  } else {
    impl_->min_scan_z_ = sdf->GetElement("min_scan_z")->Get<double>();
  }

  if (!sdf->HasElement("max_scan_x")) {
    RCLCPP_INFO(
      logger, "Gidmap plugin missing <max_scan_x> wasn't set,"
      "therefore it's been set as '5.0'.");
    impl_->max_scan_x_ = 5.0;
  } else {
    impl_->max_scan_x_ = sdf->GetElement("max_scan_x")->Get<double>();
  }

  if (!sdf->HasElement("max_scan_y")) {
    RCLCPP_INFO(
      logger, "Gidmap plugin missing <max_scan_y> wasn't set,"
      "therefore it's been set as '5.0'.");
    impl_->max_scan_y_ = 5.0;
  } else {
    impl_->max_scan_y_ = sdf->GetElement("max_scan_y")->Get<double>();
  }

  if (!sdf->HasElement("max_scan_z")) {
    RCLCPP_INFO(
      logger, "Gidmap plugin missing <max_scan_z> wasn't set,"
      "therefore it's been set as '5.0'.");
    impl_->max_scan_z_ = 5.0;
  } else {
    impl_->max_scan_z_ = sdf->GetElement("max_scan_z")->Get<double>();
  }

  if (!sdf->HasElement("min_height")) {
    RCLCPP_INFO(
      logger, "Gidmap plugin missing <min_height> wasn't set,"
      "therefore it's been set as '0.1'.");
    impl_->min_height_ = 0.1;
  } else {
    impl_->min_height_ = sdf->GetElement("min_height")->Get<double>();
  }

  if (!sdf->HasElement("max_height")) {
    RCLCPP_INFO(
      logger, "Gidmap plugin missing <max_height> wasn't set,"
      "therefore it's been set as '1.0'.");
    impl_->max_height_ = 1.0;
  } else {
    impl_->max_height_ = sdf->GetElement("max_height")->Get<double>();
  }

  if (!sdf->HasElement("resolution")) {
    RCLCPP_INFO(
      logger, "Gidmap plugin missing <resolution> wasn't set,"
      "therefore it's been set as '0.1'.");
    impl_->resolution_ = 0.1;
  } else {
    impl_->resolution_ = sdf->GetElement("resolution")->Get<double>();
  }
  impl_->world_ = _parent;
}

void GazeboRosGridmap::OnUpdate()
{
  if (!impl_->gridmap_created_) {
    impl_->gridmap_created_ = true;
    create_gridmap();
  }

  if (!impl_->octomap_created_) {
    impl_->octomap_created_ = true;
    create_octomap();
  }
  // Do something every simulation iteration
}


double GazeboRosGridmap::get_surface(
  const ignition::math::Vector3d & central_point,
  const double min_z, const double max_z,
  const double resolution,
  gazebo::physics::RayShapePtr ray)
{
  ignition::math::Vector3d start_point = central_point;
  ignition::math::Vector3d end_point = central_point;
  start_point.Z() = min_z;
  end_point.Z() = max_z;

  double dist = 0.0;
  std::string entity_name;
  std::string current_entity;

  ray->SetPoints(start_point, end_point);
  ray->GetIntersection(dist, current_entity);

  return start_point.Z() + dist;
}


bool GazeboRosGridmap::is_obstacle(
  const ignition::math::Vector3d & central_point, double surface,
  const double min_z, const double max_z,
  const double resolution,
  gazebo::physics::RayShapePtr ray)
{
  ignition::math::Vector3d start_point = central_point;
  ignition::math::Vector3d end_point = central_point;

  std::string entity;
  double dist;

  for (double z = min_z; z < max_z; z = z + resolution) {
    start_point.X() = central_point.X() - resolution / 2.0;
    start_point.Y() = central_point.Y();
    start_point.Z() = surface + z;

    end_point.X() = end_point.X() - resolution / 2.0;
    start_point.Y() = central_point.Y();
    start_point.Z() = surface + z;

    ray->SetPoints(start_point, end_point);
    ray->GetIntersection(dist, entity);

    if (dist < resolution) {
      return true;
    }
  }

  return false;
}

void GazeboRosGridmap::create_gridmap()
{
  double & resolution = impl_->resolution_;
  double & max_height = impl_->max_height_;
  double & min_height = impl_->min_height_;
  double & center_x = impl_->center_x_;
  double & center_y = impl_->center_y_;
  double & min_x = impl_->min_scan_x_;
  double & min_y = impl_->min_scan_y_;
  double & min_z = impl_->min_scan_z_;
  double & max_x = impl_->max_scan_x_;
  double & max_y = impl_->max_scan_y_;
  double & max_z = impl_->max_scan_z_;

  double size_x = max_x - min_x;
  double size_y = max_y - min_y;

  std::cerr << "Creating gridmap (" << size_x << " x " << size_y << ")" << std::endl;

  impl_->gridmap_.setFrameId("map");
  impl_->gridmap_.setGeometry(
    grid_map::Length(size_x, size_y), resolution,
    grid_map::Position(center_x, center_y));

  gazebo::physics::PhysicsEnginePtr engine = impl_->world_->Physics();
  engine->InitForThread();
  gazebo::physics::RayShapePtr ray =
    boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
    engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

  // Surface
  // iterate the gridmap and fill each cell
  for (grid_map::GridMapIterator grid_iterator(impl_->gridmap_); !grid_iterator.isPastEnd();
    ++grid_iterator)
  {
    // get the value at the iterator
    grid_map::Position current_pos;
    impl_->gridmap_.getPosition(*grid_iterator, current_pos);
    ignition::math::Vector3d point(current_pos.x(), current_pos.y(), 0);

    // get the height at this point
    impl_->gridmap_.atPosition("elevation", current_pos) =
      get_surface(point, min_z, max_z, resolution, ray);
  }

  std::cout << "Surface completed " << std::endl;

  // Obstacles
  for (grid_map::GridMapIterator obs_it(impl_->gridmap_); !obs_it.isPastEnd(); ++obs_it) {
    // get the value at the iterator
    grid_map::Position current_pos;
    impl_->gridmap_.getPosition(*obs_it, current_pos);
    ignition::math::Vector3d point(current_pos.x(), current_pos.y(), 0);

    double surface = impl_->gridmap_.atPosition("elevation", current_pos);
    if (is_obstacle(point, surface, min_height, max_height, resolution, ray)) {
      impl_->gridmap_.atPosition("occupancy", current_pos) = 254;
    } else {
      impl_->gridmap_.atPosition("occupancy", current_pos) = 1.0;  // free space
    }
  }

  std::cout << "Obstacles completed " << std::endl;

  std::unique_ptr<grid_map_msgs::msg::GridMap> message;
  message = grid_map::GridMapRosConverter::toMessage(impl_->gridmap_);
  impl_->gridmap_pub_->publish(std::move(message));
}
// @brief a ray is casted from the bottom of the cube to the top by the center
//      to check occupancy
// @param central_point center of the voxel to check
// @param resolution size of the cube to check
// @returns true if the cube centered in central_point and size resolution
//          is occupied, false otherwise
bool GazeboRosGridmap::voxel_is_obstacle(
  const ignition::math::Vector3d & central_point,
  const double resolution,
  gazebo::physics::RayShapePtr ray)
{
  ignition::math::Vector3d start_point = central_point;
  ignition::math::Vector3d end_point = central_point;

  // bottom point
  start_point.Z() = start_point.Z() - resolution/2;
  end_point.Z() = end_point.Z() + resolution/2;
  
  std::string entity;
  double dist;

  ray->SetPoints(start_point, end_point);
  ray->GetIntersection(dist, entity);

  if (dist < resolution){
    return true;
  }

  return false;
}

void GazeboRosGridmap::create_octomap(){

  gazebo::physics::PhysicsEnginePtr engine = impl_->world_->Physics();
  engine->InitForThread();
  gazebo::physics::RayShapePtr ray =
    boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
    engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

  double & resolution = impl_->resolution_;
  // double & max_height = impl_->max_height_;
  // double & min_height = impl_->min_height_;
  double & min_x = impl_->min_scan_x_;
  double & min_y = impl_->min_scan_y_;
  double & min_z = impl_->min_scan_z_;
  double & max_x = impl_->max_scan_x_;
  double & max_y = impl_->max_scan_y_;
  double & max_z = impl_->max_scan_z_;

  impl_->octomap_ = std::make_unique<octomap::OcTree>(resolution);

  std::cout << "Creating octomap" << std::endl;

  for (double i = min_x; i < max_x; i += resolution){
    for (double j = min_y; j < max_y; j += resolution){
      for (double k = min_z; k < max_z; k += resolution){
        if (voxel_is_obstacle(ignition::math::Vector3d(i, j, k), resolution, ray)){
          impl_->octomap_->updateNode(i, j, k, true);
        }
      }
    }
  }

  std::cout << "Octomap completed" << std::endl;

  octomap_msgs::msg::Octomap message;

  octomap_msgs::fullMapToMsg(*(impl_->octomap_), message);
  message.header.frame_id = "map";
  message.header.stamp = impl_->ros_node_->get_clock()->now();
  impl_->octomap_pub_->publish(message);

}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(GazeboRosGridmap)
}  // namespace gazebo_plugins
