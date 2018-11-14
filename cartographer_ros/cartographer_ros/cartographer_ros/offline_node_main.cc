/*
 * Copyright 2017 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*
 *
-configuration_directory
${HOME}/cartographer/carto_modify/cartographer_ros/cartographer_ros/configuration_files/
-configuration_basenames
sensor.lua
-bag_filenames
/media/tb/work/bagfile/sensor_9.7_a.bag
-urdf_filenames
${HOME}/cartographer/carto_cz/cartographer_ros/cartographer_ros/urdf/sensor.urdf


build & run cartographer_offline_node, should do edit configurations ( offline_node.cc )
T1:
roscore
T2:
cd ../carto_modify
source cmake-build-relwithdebinfo/devel/setup.bash
rosparam load cartographer_ros/cartographer_ros/urdf/sensor.urdf /robot_description
rosrun robot_state_publisher robot_state_publisher
T3:
cd ../carto_modify
source cmake-build-relwithdebinfo/devel/setup.bash
rviz -d cartographer_ros/cartographer_ros/configuration_files/sensor.rviz
 *
 */
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/offline_node.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "ros/ros.h"

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  ::ros::init(argc, argv, "cartographer_offline_node");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;

  const cartographer_ros::MapBuilderFactory map_builder_factory =
      [](const ::cartographer::mapping::proto::MapBuilderOptions&
             map_builder_options) {
        return absl::make_unique< ::cartographer::mapping::MapBuilder>(
            map_builder_options);
      };

  cartographer_ros::RunOfflineNode(map_builder_factory);

  ::ros::shutdown();
}
