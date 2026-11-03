// motion_server: expone servicios ROS 2 simples sobre MoveIt 2 (MoveGroupInterface)
// para que nodos Python (vision, pick & place) muevan el JetCobot sin depender de
// la API C++ de MoveIt.
//
//   /jc/move_to_pose   (jc_interfaces/srv/MoveToPose)   pose del TCP (articular u cartesiana)
//   /jc/move_to_named  (jc_interfaces/srv/MoveToNamed)  estado del SRDF: home, ready, observe
//   /jc/set_gripper    (jc_interfaces/srv/SetGripper)   apertura 0..1
//
// Referencias: MoveIt 2 Humble, tutorial "Move Group C++ Interface"
// https://moveit.picknik.ai/humble/doc/examples/move_group_interface/move_group_interface_tutorial.html

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "jc_interfaces/srv/move_to_pose.hpp"
#include "jc_interfaces/srv/move_to_named.hpp"
#include "jc_interfaces/srv/set_gripper.hpp"

using moveit::planning_interface::MoveGroupInterface;
using namespace std::chrono_literals;

class MotionServer
{
public:
  explicit MotionServer(const rclcpp::Node::SharedPtr & node)
  : node_(node),
    arm_(node, "arm"),
    gripper_(node, "gripper"),
    tf_buffer_(node->get_clock()),
    tf_listener_(tf_buffer_)
  {
    default_vel_ = param("default_velocity_scaling", 0.3);
    default_acc_ = param("default_acceleration_scaling", 0.3);
    eef_step_ = param("cartesian_eef_step", 0.005);
    min_fraction_ = param("cartesian_min_fraction", 0.95);
    finger_max_ = param("finger_max_opening", 0.017);
    add_table_ = param("add_table", true);

    arm_.setEndEffectorLink("tcp");
    arm_.setPlanningTime(5.0);
    arm_.setNumPlanningAttempts(5);
    arm_.setGoalPositionTolerance(0.002);
    arm_.setGoalOrientationTolerance(0.01);

    cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    srv_pose_ = node_->create_service<jc_interfaces::srv::MoveToPose>(
      "/jc/move_to_pose",
      std::bind(&MotionServer::onMoveToPose, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, cb_group_);
    srv_named_ = node_->create_service<jc_interfaces::srv::MoveToNamed>(
      "/jc/move_to_named",
      std::bind(&MotionServer::onMoveToNamed, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, cb_group_);
    srv_grip_ = node_->create_service<jc_interfaces::srv::SetGripper>(
      "/jc/set_gripper",
      std::bind(&MotionServer::onSetGripper, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, cb_group_);

    if (add_table_) {addTable();}
    RCLCPP_INFO(node_->get_logger(), "motion_server listo. Marco de planificacion: %s, TCP: %s",
      arm_.getPlanningFrame().c_str(), arm_.getEndEffectorLink().c_str());
  }

private:
  // Con automatically_declare_parameters_from_overrides(true) un parametro pasado desde
  // el launch ya esta declarado: se lee; si no, se declara con su valor por defecto.
  template<typename T>
  T param(const std::string & name, const T & def)
  {
    if (!node_->has_parameter(name)) {node_->declare_parameter<T>(name, def);}
    return node_->get_parameter(name).get_value<T>();
  }

  double scale(double requested) const
  {
    return (requested > 0.0 && requested <= 1.0) ? requested : default_vel_;
  }

  void addTable()
  {
    moveit_msgs::msg::CollisionObject mesa;
    mesa.header.frame_id = arm_.getPlanningFrame();
    mesa.id = "mesa";
    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = {0.80, 0.80, 0.02};
    geometry_msgs::msg::Pose p;
    p.position.x = 0.10;
    p.position.z = -0.0115;  // superficie a z = -1.5 mm (la base apoya en z = 0)
    p.orientation.w = 1.0;
    mesa.primitives.push_back(box);
    mesa.primitive_poses.push_back(p);
    mesa.operation = mesa.ADD;
    scene_.applyCollisionObject(mesa);
  }

  bool planAndExecute(MoveGroupInterface & group, std::string & msg)
  {
    MoveGroupInterface::Plan plan;
    auto ok = group.plan(plan);
    if (ok != moveit::core::MoveItErrorCode::SUCCESS) {
      msg = "Fallo la planificacion (codigo " + std::to_string(ok.val) + ")";
      return false;
    }
    auto ex = group.execute(plan);
    if (ex != moveit::core::MoveItErrorCode::SUCCESS) {
      msg = "Fallo la ejecucion (codigo " + std::to_string(ex.val) + ")";
      return false;
    }
    msg = "OK";
    return true;
  }

  void onMoveToPose(
    const std::shared_ptr<jc_interfaces::srv::MoveToPose::Request> req,
    std::shared_ptr<jc_interfaces::srv::MoveToPose::Response> res)
  {
    const double vel = scale(req->velocity_scaling);
    arm_.setStartStateToCurrentState();
    arm_.setMaxVelocityScalingFactor(vel);
    arm_.setMaxAccelerationScalingFactor(std::min(vel, default_acc_));

    // Llevar la pose objetivo al marco de planificacion (world)
    geometry_msgs::msg::PoseStamped target = req->target;
    if (target.header.frame_id.empty()) {target.header.frame_id = arm_.getPlanningFrame();}
    try {
      target = tf_buffer_.transform(target, arm_.getPlanningFrame(), tf2::durationFromSec(1.0));
    } catch (const tf2::TransformException & e) {
      res->success = false;
      res->message = std::string("TF: ") + e.what();
      return;
    }

    if (!req->cartesian) {
      arm_.setPoseTarget(target, "tcp");
      res->success = planAndExecute(arm_, res->message);
      arm_.clearPoseTargets();
      return;
    }

    // Trayectoria cartesiana (linea recta del TCP)
    std::vector<geometry_msgs::msg::Pose> waypoints{target.pose};
    moveit_msgs::msg::RobotTrajectory traj_msg;
    const double fraction = arm_.computeCartesianPath(waypoints, eef_step_, 0.0, traj_msg);
    if (fraction < min_fraction_) {
      res->success = false;
      res->message = "Trayectoria cartesiana incompleta: " + std::to_string(fraction * 100.0) + " %";
      return;
    }
    // Reparametrizar en el tiempo con el escalado pedido (TOTG)
    robot_trajectory::RobotTrajectory rt(arm_.getRobotModel(), "arm");
    rt.setRobotTrajectoryMsg(*arm_.getCurrentState(), traj_msg);
    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    totg.computeTimeStamps(rt, vel, std::min(vel, default_acc_));
    rt.getRobotTrajectoryMsg(traj_msg);
    auto ex = arm_.execute(traj_msg);
    res->success = (ex == moveit::core::MoveItErrorCode::SUCCESS);
    res->message = res->success ? "OK (cartesiana)" : "Fallo la ejecucion cartesiana";
  }

  void onMoveToNamed(
    const std::shared_ptr<jc_interfaces::srv::MoveToNamed::Request> req,
    std::shared_ptr<jc_interfaces::srv::MoveToNamed::Response> res)
  {
    arm_.setStartStateToCurrentState();
    arm_.setMaxVelocityScalingFactor(scale(req->velocity_scaling));
    arm_.setMaxAccelerationScalingFactor(default_acc_);
    if (!arm_.setNamedTarget(req->name)) {
      res->success = false;
      res->message = "Estado desconocido en el SRDF: " + req->name;
      return;
    }
    res->success = planAndExecute(arm_, res->message);
  }

  void onSetGripper(
    const std::shared_ptr<jc_interfaces::srv::SetGripper::Request> req,
    std::shared_ptr<jc_interfaces::srv::SetGripper::Response> res)
  {
    const double o = std::clamp(req->opening, 0.0, 1.0) * finger_max_;
    gripper_.setStartStateToCurrentState();
    gripper_.setMaxVelocityScalingFactor(1.0);
    gripper_.setJointValueTarget(std::map<std::string, double>{
        {"gripper_left_joint", o}, {"gripper_right_joint", o}});
    res->success = planAndExecute(gripper_, res->message);
  }

  rclcpp::Node::SharedPtr node_;
  MoveGroupInterface arm_;
  MoveGroupInterface gripper_;
  moveit::planning_interface::PlanningSceneInterface scene_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::Service<jc_interfaces::srv::MoveToPose>::SharedPtr srv_pose_;
  rclcpp::Service<jc_interfaces::srv::MoveToNamed>::SharedPtr srv_named_;
  rclcpp::Service<jc_interfaces::srv::SetGripper>::SharedPtr srv_grip_;
  double default_vel_, default_acc_, eef_step_, min_fraction_, finger_max_;
  bool add_table_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts;
  opts.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("motion_server", opts);

  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4);
  exec.add_node(node);
  std::thread spinner([&exec]() {exec.spin();});

  auto server = std::make_shared<MotionServer>(node);
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
