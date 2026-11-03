// trajectory_client: cliente C++ de control_msgs/action/FollowJointTrajectory (Dia 3).
// Envia una trayectoria articular por puntos de paso (en grados) con perfil cubico
// (velocidad cero en los extremos) y reporta la realimentacion.
// Funciona igual con ros2_control (mock/Gazebo) y con jc_driver/jetcobot_bridge (real).
//   ros2 run jc_motion trajectory_client --ros-args -p segment_time:=2.5
#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

using FJT = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<FJT>;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("trajectory_client");
  const double seg = node->declare_parameter("segment_time", 2.5);
  auto client = rclcpp_action::create_client<FJT>(node, "/arm_controller/follow_joint_trajectory");
  if (!client->wait_for_action_server(std::chrono::seconds(20))) {
    RCLCPP_ERROR(node->get_logger(), "Servidor de accion no disponible");
    return 1;
  }
  const std::vector<std::vector<double>> wps_deg = {
    {0, 0, -90, 0, 0, 0}, {25, 10, -100, 0, 0, 20}, {-25, 10, -100, 0, 0, -20}, {0, 0, -90, 0, 0, 0}};

  FJT::Goal goal;
  goal.trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  for (size_t k = 0; k < wps_deg.size(); ++k) {
    trajectory_msgs::msg::JointTrajectoryPoint p;
    for (double a : wps_deg[k]) {
      p.positions.push_back(a * M_PI / 180.0);
      p.velocities.push_back(0.0);  // velocidad nula en cada punto de paso -> perfil cubico por tramo
    }
    p.time_from_start = rclcpp::Duration::from_seconds(seg * static_cast<double>(k + 1));
    goal.trajectory.points.push_back(p);
  }

  rclcpp_action::Client<FJT>::SendGoalOptions opts;
  opts.feedback_callback = [node](GoalHandle::SharedPtr, const std::shared_ptr<const FJT::Feedback> fb) {
      if (!fb->actual.positions.empty()) {
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "J1 actual = %.1f deg",
          fb->actual.positions[0] * 180.0 / M_PI);
      }
    };
  auto gh_future = client->async_send_goal(goal, opts);
  if (rclcpp::spin_until_future_complete(node, gh_future) != rclcpp::FutureReturnCode::SUCCESS ||
    !gh_future.get())
  {
    RCLCPP_ERROR(node->get_logger(), "Meta rechazada");
    return 1;
  }
  auto res_future = client->async_get_result(gh_future.get());
  rclcpp::spin_until_future_complete(node, res_future);
  auto res = res_future.get();
  RCLCPP_INFO(node->get_logger(), "Resultado: error_code = %d (0 = SUCCESSFUL)", res.result->error_code);
  rclcpp::shutdown();
  return res.result->error_code == 0 ? 0 : 2;
}
