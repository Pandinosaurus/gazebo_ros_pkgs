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


#include <gazebo_plugins/gazebo_ros_imu_sensor.hpp>
#include <gazebo_ros/conversions.hpp>
#include <gazebo_ros/utils.hpp>

#include <iostream>
#include <memory>
#include <string>

namespace gazebo_plugins
{

class GazeboRosImuSensorPrivate
{
public:
  gazebo_ros::Node::SharedPtr rosnode_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  sensor_msgs::msg::Imu::SharedPtr msg_;
  gazebo::sensors::ImuSensorPtr sensor_;
  gazebo::event::ConnectionPtr sensor_update_event_;
  std::string frame_id_;

  void OnUpdate();
};

GazeboRosImuSensor::GazeboRosImuSensor()
: impl_(std::make_unique<GazeboRosImuSensorPrivate>())
{
}

GazeboRosImuSensor::~GazeboRosImuSensor()
{
}

void GazeboRosImuSensor::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  impl_->rosnode_ = gazebo_ros::Node::Create("gazebo_ros_imu_sensor", _sdf);

  impl_->sensor_ = std::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(_sensor);
  if (!impl_->sensor_) {
    RCLCPP_ERROR(impl_->rosnode_->get_logger(), "Parent is not an imu sensor. Exiting.");
    return;
  }

  impl_->pub_ = impl_->rosnode_->create_publisher<sensor_msgs::msg::Imu>("~/out");

  // Create message to be reused
  auto msg = std::make_shared<sensor_msgs::msg::Imu>();

  // Fill covariances
  using SNT = gazebo::sensors::SensorNoiseType;
  msg->angular_velocity_covariance[0] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_ANGVEL_X_NOISE_RADIANS_PER_S));
  msg->angular_velocity_covariance[4] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_ANGVEL_Y_NOISE_RADIANS_PER_S));
  msg->angular_velocity_covariance[8] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_ANGVEL_Z_NOISE_RADIANS_PER_S));
  msg->linear_acceleration_covariance[0] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_LINACC_X_NOISE_METERS_PER_S_SQR));
  msg->linear_acceleration_covariance[4] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_LINACC_Y_NOISE_METERS_PER_S_SQR));
  msg->linear_acceleration_covariance[8] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::IMU_LINACC_Z_NOISE_METERS_PER_S_SQR));

  impl_->msg_ = msg;

  impl_->sensor_update_event_ = impl_->sensor_->ConnectUpdated(
    std::bind(&GazeboRosImuSensorPrivate::OnUpdate, impl_.get()));
}

void GazeboRosImuSensorPrivate::OnUpdate()
{
  // TODO(ironmig): frame id
  // TODO(ironmig): covariances

  // Fill message with latest sensor data
  msg_->header.frame_id = frame_id_;
  msg_->header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(
    sensor_->LastUpdateTime());
  msg_->orientation =
    gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(sensor_->Orientation());
  msg_->angular_velocity = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(
    sensor_->AngularVelocity());
  msg_->linear_acceleration = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(
    sensor_->LinearAcceleration());

  // Publish message
  pub_->publish(msg_);
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboRosImuSensor)

}  // namespace gazebo_plugins
