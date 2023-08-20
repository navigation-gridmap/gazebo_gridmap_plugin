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


#include <Eigen/Core>

#include <octomap/octomap.h>

#include <memory>
#include <string>
#include <utility>


#include <gazebo/physics/Model.hh>
#include <gazebo_plugins/gazebo_ros_gridmap.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "grid_map_ros/grid_map_ros.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"

#include <octomap_msgs/msg/octomap.hpp>
#include "octomap_msgs/conversions.h"
#include "octomap_ros/conversions.hpp"

namespace gazebo_plugins
{

typedef struct
{
  grid_map::Position pos;
  double last_height {0.0};
} TFloodCell;

/// Class to hold private data members (PIMPL pattern)
class GazeboRosGridmapPrivate
{
public:
  GazeboRosGridmapPrivate()
  : gridmap_() {}

  /// Connection to world update event. Callback is called while this is alive.
  gazebo::event::ConnectionPtr update_connection_;

  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  gazebo::physics::WorldPtr world_;
  bool surface_created_{false};
  bool occupancy_created_{false};

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
  if (!impl_->surface_created_) {
    impl_->surface_created_ = true;
    create_gridmap();
  }

  if (!impl_->octomap_created_) {
    impl_->octomap_created_ = true;
    create_octomap();
  }

  if (!impl_->occupancy_created_) {
    impl_->occupancy_created_ = true;
    create_occupancy();
  }
  // Do something every simulation iteration
}


bool GazeboRosGridmap::get_surface(
  const ignition::math::Vector3d & central_point,
  const double min_z, const double max_z,
  const double resolution, gazebo::physics::RayShapePtr ray,
  double & height)
{
  ignition::math::Vector3d start_point = central_point;
  ignition::math::Vector3d end_point = central_point;
  start_point.Z() = max_z;
  end_point.Z() = min_z;

  double dist = 0.0;
  std::string entity_name;
  std::string current_entity;

  ray->SetPoints(start_point, end_point);
  ray->GetIntersection(dist, current_entity);

  if (current_entity == "") {
    return false;
  } else {
    height = start_point.Z() - dist;
    return true;
  }
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

  start_point.Z() = surface + min_z;

  end_point.Z() = surface + max_z;

  ray->SetPoints(start_point, end_point);
  ray->GetIntersection(dist, entity);

  if (dist < max_z - min_z) {
    return true;
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

  std::cerr << "Creating surface" << std::endl;
  impl_->gridmap_.setFrameId("map");
  impl_->gridmap_.setGeometry(
    grid_map::Length(size_x, size_y), resolution,
    grid_map::Position(center_x, center_y));

  gazebo::physics::PhysicsEnginePtr engine = impl_->world_->Physics();
  engine->InitForThread();
  gazebo::physics::RayShapePtr ray =
    boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
    engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

  impl_->gridmap_.add("elevation");
  impl_->gridmap_.add("occupancy");
  grid_map::Position current_pos(center_x, center_y);

  double height = 0.0;
  ignition::math::Vector3d point(current_pos.x(), current_pos.y(), 0);
  if (get_surface(point, min_z, max_z, resolution, ray, height)) {
    impl_->gridmap_.atPosition("elevation", current_pos) = height;
  }

  flood(current_pos, height, resolution, ray);

  create_occupancy();


  std::cout << "Surface completed " << std::endl;

  std::unique_ptr<grid_map_msgs::msg::GridMap> message;
  message = grid_map::GridMapRosConverter::toMessage(impl_->gridmap_);
  impl_->gridmap_pub_->publish(std::move(message));
}

void
GazeboRosGridmap::process_neighbour(
  const grid_map::Index & pos, double current_height,
  double resolution, std::stack<grid_map::Index> & pending_poses,
  gazebo::physics::RayShapePtr ray, MatrixXf & em)
{
  const double thr = resolution * 1.0;
  double height;

  grid_map::Position metric_pos;

  if (!impl_->gridmap_.getPosition(pos, metric_pos)) {
    return;
  }

  grid_map::Index debug_idx;
  grid_map::Position debug_pos(-11.0, 13.10);
  impl_->gridmap_.getIndex(debug_pos, debug_idx);

  ignition::math::Vector3d point(metric_pos.x(), metric_pos.y(), 0.0);

  bool there_is_surface = get_surface(
    point, current_height - (thr), current_height + (thr), resolution, ray, height);
  bool already_visited = !std::isnan(em(pos(0), pos(1)));
  double diff = height - em(pos(0), pos(1));


  if (there_is_surface && (!already_visited)) {
    em(pos(0), pos(1)) = height;
    pending_poses.push(pos);
  } else if (there_is_surface && diff > thr) {
    em(pos(0), pos(1)) = height;

    grid_map::Index n_pos(pos);
    n_pos(0) = n_pos(0) + 1;
    pending_poses.push(n_pos);

    grid_map::Index w_pos(pos);
    w_pos(1) = w_pos(1) + 1;
    pending_poses.push(w_pos);

    grid_map::Index e_pos(pos);
    e_pos(1) = e_pos(1) - 1;
    pending_poses.push(e_pos);

    grid_map::Index s_pos(pos);
    s_pos(0) = s_pos(0) - 1;
    pending_poses.push(s_pos);
  }
}

void
GazeboRosGridmap::flood(
  const grid_map::Position & current_pos,
  double current_height, double resolution, gazebo::physics::RayShapePtr ray)
{
  MatrixXf em(impl_->gridmap_.getSize()(0), impl_->gridmap_.getSize()(1));
  for (auto i = 0; i < impl_->gridmap_.getSize()(0); i++) {
    for (auto j = 0; j < impl_->gridmap_.getSize()(1); j++) {
      em(i, j) = NAN;
    }
  }

  std::stack<grid_map::Index> pending_poses;

  grid_map::Index init_index;
  if (!impl_->gridmap_.getIndex(current_pos, init_index)) {
    return;
  }

  pending_poses.push(init_index);
  em(init_index(0), init_index(1)) = current_height;

  int counter_flood = 0;
  while (!pending_poses.empty()) {
    grid_map::Index cell = pending_poses.top();
    pending_poses.pop();

    process_neighbour(cell, current_height, resolution, pending_poses, ray, em);
    current_height = em(cell(0), cell(1));

    // Check N cell
    grid_map::Index n_pos(cell);
    n_pos(0) = n_pos(0) + 1;
    process_neighbour(n_pos, current_height, resolution, pending_poses, ray, em);

    grid_map::Index w_pos(cell);
    w_pos(1) = w_pos(1) + 1;
    process_neighbour(w_pos, current_height, resolution, pending_poses, ray, em);

    grid_map::Index e_pos(cell);
    e_pos(1) = e_pos(1) - 1;
    process_neighbour(e_pos, current_height, resolution, pending_poses, ray, em);

    grid_map::Index s_pos(cell);
    s_pos(0) = s_pos(0) - 1;
    process_neighbour(s_pos, current_height, resolution, pending_poses, ray, em);

    counter_flood++;
  }

  for (auto i = 0; i < impl_->gridmap_.getSize()(0); i++) {
    for (auto j = 0; j < impl_->gridmap_.getSize()(1); j++) {
      grid_map::Index map_index(i, j);
      impl_->gridmap_.at("elevation", map_index) = em(i, j);
    }
  }

  // Fill holes
  double height = 0.0;
  for (grid_map::GridMapIterator grid_iterator(impl_->gridmap_); !grid_iterator.isPastEnd();
    ++grid_iterator)
  {
    grid_map::Position pos;
    impl_->gridmap_.getPosition(*grid_iterator, pos);

    double cell_height = impl_->gridmap_.atPosition("elevation", pos);
    if (cell_height == NAN) {
      impl_->gridmap_.atPosition("elevation", pos) = height;
    } else {
      height = cell_height;
    }
  }
}

void
GazeboRosGridmap::create_occupancy()
{
  std::cerr << "Creating occupancy" << std::endl;

  float res = static_cast<float>(impl_->resolution_);
  double & max_height = impl_->max_height_;
  double & min_height = impl_->min_height_;

  gazebo::physics::PhysicsEnginePtr engine = impl_->world_->Physics();
  engine->InitForThread();
  gazebo::physics::RayShapePtr ray =
    boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
    engine->CreateShape("ray", gazebo::physics::CollisionPtr()));


  for (auto i = 0; i < impl_->gridmap_.getSize()(0); i++) {
    for (auto j = 0; j < impl_->gridmap_.getSize()(1); j++) {
      grid_map::Index idx(i, j);
      impl_->gridmap_.at("occupancy", idx) = 1.0;
    }
  }

  for (auto i = 0; i < impl_->gridmap_.getSize()(0); i++) {
    for (auto j = 0; j < impl_->gridmap_.getSize()(1); j++) {
      grid_map::Index idx(i, j);
      float height = impl_->gridmap_.at("elevation", idx);

      float diff_n(0.0), diff_s(0.0), diff_w(0.0), diff_e(0.0);

      // Check N cell
      grid_map::Index n_pos(idx);
      n_pos(0) = n_pos(0) + 1;
      if (impl_->gridmap_.isValid(n_pos, "elevation")) {
        diff_n = abs(impl_->gridmap_.at("elevation", n_pos) - height);
      }

      grid_map::Index w_pos(idx);
      w_pos(1) = w_pos(1) + 1;
      if (impl_->gridmap_.isValid(w_pos, "elevation")) {
        diff_w = abs(impl_->gridmap_.at("elevation", w_pos) - height);
      }

      grid_map::Index e_pos(idx);
      e_pos(1) = e_pos(1) - 1;
      if (impl_->gridmap_.isValid(e_pos, "elevation")) {
        diff_e = abs(impl_->gridmap_.at("elevation", e_pos) - height);
      }

      grid_map::Index s_pos(idx);
      s_pos(0) = s_pos(0) - 1;
      if (impl_->gridmap_.isValid(s_pos, "elevation")) {
        diff_s = abs(impl_->gridmap_.at("elevation", s_pos) - height);
      }

      grid_map::Position pos;
      impl_->gridmap_.getPosition(n_pos, pos);
      ignition::math::Vector3d point(pos(0), pos(1), 0.0);

      bool obstacle = is_obstacle(point, height, min_height, max_height, res, ray);

      if (obstacle || diff_n > res || diff_s > res || diff_w > res || diff_e > res) {
        impl_->gridmap_.at("occupancy", idx) = 254.0;
      }
    }
  }

  std::cerr << "Occupancy created" << std::endl;
}


bool ray_collision(
  const ignition::math::Vector3d & start,
  const ignition::math::Vector3d & end,
  const double min_dist,
  gazebo::physics::RayShapePtr ray)
{
  std::string entity;
  double dist;

  ray->SetPoints(start, end);
  ray->GetIntersection(dist, entity);

  return dist < min_dist;
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
  ignition::math::Vector3d start_point_x = central_point;
  ignition::math::Vector3d end_point_x = central_point;
  ignition::math::Vector3d start_point_y = central_point;
  ignition::math::Vector3d end_point_y = central_point;
  ignition::math::Vector3d start_point_z = central_point;
  ignition::math::Vector3d end_point_z = central_point;

  // X line
  start_point_x.X() = start_point_x.X() - resolution / 2;
  end_point_x.X() = end_point_x.X() + resolution / 2;
  // Y line
  start_point_y.Y() = start_point_y.Y() - resolution / 2;
  end_point_y.Y() = end_point_y.Y() + resolution / 2;
  // Z line
  start_point_z.Z() = start_point_z.Z() - resolution / 2;
  end_point_z.Z() = end_point_z.Z() + resolution / 2;

  bool collision_x = ray_collision(start_point_x, end_point_x, resolution, ray);
  bool collision_y = ray_collision(start_point_y, end_point_y, resolution, ray);
  bool collision_z = ray_collision(start_point_z, end_point_z, resolution, ray);
  // returns true if is there a collision in any axis
  return collision_x || collision_y || collision_z;
}

void GazeboRosGridmap::create_octomap()
{
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
  double filter_min;

  impl_->octomap_ = std::make_unique<octomap::OcTree>(resolution);

  std::cout << "Creating octomap" << std::endl;

  for (double i = min_x; i < max_x; i += resolution) {
    for (double j = min_y; j < max_y; j += resolution) {
      for (double k = min_z; k < max_z; k += resolution) {
        if (voxel_is_obstacle(ignition::math::Vector3d(i, j, k), resolution, ray)) {
          impl_->octomap_->updateNode(i, j, k, true);
        }
      }
    }
  }

  std::cout << "Octomap completed" << std::endl;

  // filter octomap
  // iterate the gridmap and set the elevation as max Z possible
  /*
  for (grid_map::GridMapIterator grid_iterator(impl_->gridmap_); !grid_iterator.isPastEnd();
    ++grid_iterator)
  {
    // get the value at the iterator
    grid_map::Position current_pos;
    impl_->gridmap_.getPosition(*grid_iterator, current_pos);
    filter_min = impl_->gridmap_.atPosition("elevation", current_pos);

    for (double k = min_z; k < filter_min + resolution; k += resolution) {
      impl_->octomap_->deleteNode(current_pos.x(), current_pos.y(), k);
    }
  } */
  octomap_msgs::msg::Octomap message;

  octomap_msgs::fullMapToMsg(*(impl_->octomap_), message);
  message.header.frame_id = "map";
  message.header.stamp = impl_->ros_node_->get_clock()->now();
  impl_->octomap_pub_->publish(message);
}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(GazeboRosGridmap)
}  // namespace gazebo_plugins
