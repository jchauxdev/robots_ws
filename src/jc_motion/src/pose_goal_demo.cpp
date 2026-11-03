// pose_goal_demo: practica Dia 3 (C++). Planifica y ejecuta con MoveIt 2:
//  1) estado con nombre "ready"; 2) objetivo de pose del TCP;
//  3) cuadrado cartesiano de 4 cm en el plano XY; 4) regreso a "ready".
// Basado en el tutorial oficial "Move Group C++ Interface" (MoveIt 2 Humble).
#include <memory>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>

using moveit::planning_interface::MoveGroupInterface;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
    "pose_goal_demo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto logger = node->get_logger();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() {exec.spin();});

  MoveGroupInterface arm(node, "arm");
  arm.setEndEffectorLink("tcp");
  arm.setMaxVelocityScalingFactor(0.3);
  arm.setMaxAccelerationScalingFactor(0.3);
  arm.setPlanningTime(5.0);

  auto go = [&](const char * what) {
      MoveGroupInterface::Plan plan;
      bool ok = static_cast<bool>(arm.plan(plan));
      RCLCPP_INFO(logger, "Plan '%s': %s", what, ok ? "OK" : "FALLO");
      if (ok) {arm.execute(plan);}
      return ok;
    };

  arm.setNamedTarget("ready");
  go("ready");

  // Pose objetivo: TCP apuntando hacia abajo (roll = pi, pitch = 0, yaw = -pi/2)
  tf2::Quaternion q;
  q.setRPY(M_PI, 0.0, -M_PI / 2.0);
  geometry_msgs::msg::Pose target;
  target.orientation.x = q.x();
  target.orientation.y = q.y();
  target.orientation.z = q.z();
  target.orientation.w = q.w();
  target.position.x = 0.18;
  target.position.y = -0.06;
  target.position.z = 0.14;
  arm.setPoseTarget(target);
  go("pose");

  // Cuadrado cartesiano de 4 cm
  std::vector<geometry_msgs::msg::Pose> wps;
  auto p = target;
  p.position.x += 0.04; wps.push_back(p);
  p.position.y += 0.04; wps.push_back(p);
  p.position.x -= 0.04; wps.push_back(p);
  p.position.y -= 0.04; wps.push_back(p);
  moveit_msgs::msg::RobotTrajectory traj;
  double fraction = arm.computeCartesianPath(wps, 0.005, 0.0, traj);
  RCLCPP_INFO(logger, "Trayectoria cartesiana: %.1f %% alcanzable", fraction * 100.0);
  if (fraction > 0.95) {arm.execute(traj);}

  arm.setNamedTarget("ready");
  go("ready");

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
