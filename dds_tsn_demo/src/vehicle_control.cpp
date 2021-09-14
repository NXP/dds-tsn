/*
 * Copyright 2021 NXP
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals; // for "ms"

// The robot controller
class VehicleController : public rclcpp::Node
{
public:
  explicit VehicleController() : Node("vehicle_control")
  {
    const std::string odomTopicName = "/odom";
    const std::string cmdTopic = "/cmd_vel";
    const size_t historyDepth = 5;
    const double poseResetPositionX = -10000;

    // subscribe to the odometry data from the Gazebo vehicle model
    sub_ = create_subscription<nav_msgs::msg::Odometry>(odomTopicName,
      historyDepth, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        vehiclePose_ = *msg;
      });

    // publish vehicle command periodically based on odometry data
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmdTopic,
      rclcpp::QoS(rclcpp::KeepLast(historyDepth)));
    timer_ = this->create_wall_timer(10ms, [this] {
      VehicleControlPub();
    });

    // reset the vehicle pose until we subscribe to the odometry topic
    vehiclePose_.pose.pose.position.x = poseResetPositionX;
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  nav_msgs::msg::Odometry vehiclePose_;

  // predefined sequence of vehicle commands to pass the moose test
  void VehicleControlPub()
  {
    struct TwistCommand {
      std::string thresholdPoint;
      double xThreshold; // x coordinate to switch to the next control command 
      float  linearX;    // linear x component of the twist command
      float  angularZ;   // angular z componennt of the twist command
    };
    // fine-tuned steering commands to pass the moose test in the simulated world
    const std::vector<struct TwistCommand> steeringCommands = {
      {"straightBeforeMooseTest", -5.0,  5.0,  0.0},
      {"leftToAvoidPedestrian",   -1.0,  5.0,  0.4},
      {"righToAvoidCar",           4.5,  5.0, -0.6},
      {"leftToStayInLane",         6.6,  5.0,  0.4},
      {"straightAfterMooseTest",   6.5, -1.5,  0.0},
      {"stopVehicle",              6.52, 0.0,  0.0},
    };

    static unsigned int stateId = 0;

    if (stateId >= steeringCommands.size()) {
      return; // finished the sequence, stop publishing
    }

    // current state twist command 
    auto &state = steeringCommands[stateId];
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = state.linearX;
    cmd.linear.y  = .0;
    cmd.linear.z  = .0;
    cmd.angular.x = .0;
    cmd.angular.y = .0;
    cmd.angular.z = state.angularZ;

    // once the x coordinate threshold is passed, switch to the next state
    auto xCoordinate = vehiclePose_.pose.pose.position.x;
    if (xCoordinate >= state.xThreshold) {
      std::ostringstream s;
      s << "reached x=" << xCoordinate << ", "
        << steeringCommands[stateId].thresholdPoint.c_str()
        << " steering command: linear.x=" << cmd.linear.x << " angular.z=" << cmd.angular.z;
      RCLCPP_INFO(get_logger(), s.str());
      stateId++;
    }

    pub_->publish(cmd);
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VehicleController>();
  rclcpp::spin(node);
  return rclcpp::shutdown() ? 0 : 1;
}
