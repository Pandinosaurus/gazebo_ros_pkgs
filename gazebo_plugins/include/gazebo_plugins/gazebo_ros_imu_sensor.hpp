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


#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_IMU_SENSOR_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_IMU_SENSOR_HPP_

#include <gazebo_ros/node.hpp>
#include <gazebo/plugins/ImuSensorPlugin.hh>
#include <sensor_msgs/msg/imu.hpp>

#include <string>
#include <memory>

namespace gazebo_plugins
{

class GazeboRosImuSensorPrivate;
/**
@anchor GazeboRosImuSensor
\ref GazeboRosImuSensor is a plugin to simulate an Inertial Motion Unit sensor, the main differences from \ref GazeboRosIMU are:
- inheritance from SensorPlugin instead of ModelPlugin,
- measurements are given by gazebo ImuSensor instead of being computed by the ros plugin,
- gravity is included in inertial measurements.
*/
/** @brief Gazebo Ros imu sensor plugin. */
class GazeboRosImuSensor : public gazebo::SensorPlugin
{
public:
  /// Constructor.
  GazeboRosImuSensor();
  /// Destructor.
  virtual ~GazeboRosImuSensor();

  /// Documentation Inherited
  virtual void Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

private:
  std::unique_ptr<GazeboRosImuSensorPrivate> impl_;
};

}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_IMU_SENSOR_HPP_
