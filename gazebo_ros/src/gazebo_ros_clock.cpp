// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#include <gazebo_ros/gazebo_ros_clock.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/conversions.hpp>
#include <gazebo/physics/physics.hh>
#include <rosgraph_msgs/msg/clock.hpp>

#include <atomic>
#include <string>
#include <memory>

namespace gazebo_ros
{

class GazeboRosClockPrivate
{
public:
  gazebo_ros::Node::SharedPtr rosnode_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  gazebo::event::ConnectionPtr world_created_event_;
  gazebo::event::ConnectionPtr world_update_event_;
  gazebo::physics::WorldPtr world_;
  std::atomic_bool world_created_flag_;
  gazebo::common::Time last_publish_time_;
  gazebo::common::Time publish_period_;
  static const gazebo::common::Time DEFAULT_PUBLISH_PERIOD;

  void OnWorldCreated(std::string _world_name);
  void PublishSimTime();
};

GazeboRosClock::GazeboRosClock()
: impl_(std::make_unique<GazeboRosClockPrivate>())
{
}

GazeboRosClock::~GazeboRosClock()
{
}

void GazeboRosClock::Load(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  impl_->rosnode_ = gazebo_ros::Node::Create("gazebo_ros_clock");

  // Get publish rate from parameter if set
  impl_->publish_period_ = GazeboRosClockPrivate::DEFAULT_PUBLISH_PERIOD;
  rclcpp::Parameter rate_param;
  if (impl_->rosnode_->get_parameter("publish_rate", rate_param)) {
    if (rclcpp::ParameterType::PARAMETER_DOUBLE == rate_param.get_type()) {
      impl_->publish_period_ = gazebo::common::Time(1.0 / rate_param.as_double());
    } else if (rclcpp::ParameterType::PARAMETER_INTEGER != rate_param.get_type()) {
      impl_->publish_period_ = gazebo::common::Time(1.0 / rate_param.as_int());
    } else {
      RCLCPP_WARN(impl_->rosnode_->get_logger(),
        "Could not read value of param publish_rate [%s] as double/int, using default %ghz.",
        rate_param.value_to_string().c_str(), 1.0 / impl_->publish_period_.Double());
    }
  }

  impl_->world_created_event_ = gazebo::event::Events::ConnectWorldCreated(
    std::bind(&GazeboRosClockPrivate::OnWorldCreated, impl_.get(), std::placeholders::_1));
}

// By default publish at 10 HZ (100 millisecond period)
const gazebo::common::Time GazeboRosClockPrivate::DEFAULT_PUBLISH_PERIOD = gazebo::common::Time(0,
    1E8);

void GazeboRosClockPrivate::OnWorldCreated(std::string _world_name)
{
  if (world_created_flag_) {
    RCLCPP_WARN(
      rosnode_->get_logger(),
      "Multiple worlds created. /clock will be the sim time for the first world only.");
    return;
  }

  world_ = gazebo::physics::get_world(_world_name);
  if (!world_) {
    RCLCPP_ERROR(rosnode_->get_logger(), "Cannot get world [%s]. /clock will NOT be published",
      _world_name.c_str());
    return;
  }

  clock_pub_ = rosnode_->create_publisher<rosgraph_msgs::msg::Clock>("/clock");
  world_update_event_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosClockPrivate::PublishSimTime, this));
}

void GazeboRosClockPrivate::PublishSimTime()
{
  auto real_time = world_->RealTime();
  if (real_time - last_publish_time_ < publish_period_) {
    return;
  }
  rosgraph_msgs::msg::Clock clock;
  clock.clock = gazebo_ros::Convert<builtin_interfaces::msg::Time>(world_->SimTime());
  clock_pub_->publish(clock);
  last_publish_time_ = real_time;
}


GZ_REGISTER_SYSTEM_PLUGIN(GazeboRosClock)

}  // namespace gazebo_ros
