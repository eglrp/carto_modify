/*
 * Copyright 2016 The Cartographer Authors
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
-configuration_basename
sensor_localization.lua
-load_state_filename
/media/tb/work/bagfile/sensor_9.10_1.bag.pbstream


build & run cartographer_node, should edit configurations ( node_main.cc )
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
T4:
rosparam set /use_sim_time true
rosbag play --clock /media/tb/work/bagfile/sensor_9.7_a.bag

then, build & run cartographer_occupancy_grid_node to generate /map
(occupancy_grid_node_main.cc should not edit configurations)
 *
 */
#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"

#include "tf2_msgs/TFMessage.h" // add
#include <tf/tf.h> // add
#include <fstream> // add

DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");
DEFINE_string(
    save_pose_filename, "", "Saving pose for remapping.");  // add

namespace cartographer_ros {
namespace {

int flag = 0;

void SaveToLua(const tf2_msgs::TFMessage::ConstPtr& msg) { // add
  flag ++;

  const std::string save_pose_file =
          FLAGS_configuration_directory + "/" + FLAGS_save_pose_filename;

  if(flag==1){
    LOG(INFO) << "Saving initial transform to '" << save_pose_file << "'...";
  }

  if(msg->transforms.data()->header.frame_id == "map" &&
     msg->transforms.data()->child_frame_id == "base_link")
  {
    tf::Transform pose_tf;

    pose_tf.setOrigin(tf::Vector3(msg->transforms.data()->transform.translation.x,
                                 msg->transforms.data()->transform.translation.y,
                                 msg->transforms.data()->transform.translation.z));
    pose_tf.setRotation(tf::Quaternion(msg->transforms.data()->transform.rotation.x,
                                      msg->transforms.data()->transform.rotation.y,
                                      msg->transforms.data()->transform.rotation.z,
                                      msg->transforms.data()->transform.rotation.w));

    // ed: Quaternion to RPY
    tf::Quaternion q(
            pose_tf.getRotation().x(),
            pose_tf.getRotation().y(),
            pose_tf.getRotation().z(),
            pose_tf.getRotation().w());

    tf::Matrix3x3 m(q); // 四元数转为旋转矩阵

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw); // 旋转矩阵转为欧拉角

    int to_trajectory_id = 0;
    // ed: pitch isn't used
    std::string parsed_string ="{to_trajectory_id = "    \
                             + std::to_string(to_trajectory_id)         \
                             + ", relative_pose = { translation = { "   \
                             + std::to_string(pose_tf.getOrigin().x()) + ", " \
                             + std::to_string(pose_tf.getOrigin().y()) + ", " \
                             + std::to_string(pose_tf.getOrigin().z()) + \
                             + "}, rotation = { "\
                             + std::to_string(roll) + ","       \
                             + std::to_string(pitch) + ","      \
                             + std::to_string(yaw) + "}}}";

    std::ofstream f1(save_pose_file);
    if(!f1)
      return;

    f1 << "-- Saving the pose information for remapping." << std::endl;
    f1 << parsed_string << std::endl;

    f1.close();
  }
}

//void SaveToLua(const tf2_msgs::TFMessage::ConstPtr& tf) {
//    const std::string save_pose_file =
//            FLAGS_configuration_directory + "/" + FLAGS_save_pose_filename;
//
//    if(tf->transforms.data()->header.frame_id == "map")
//    {
//        auto temp_tf = *tf->transforms.data();
//        double temp_translation_x = temp_tf.transform.translation.x;
//        double temp_translation_y = temp_tf.transform.translation.y;
//        double temp_translation_z = temp_tf.transform.translation.z;
//        double temp_rotation_w = temp_tf.transform.rotation.w;
//        double temp_rotation_x = temp_tf.transform.rotation.x;
//        double temp_rotation_y = temp_tf.transform.rotation.y;
//        double temp_rotation_z = temp_tf.transform.rotation.z;
//
//        std::ofstream f1(save_pose_file);
//        if(!f1)
//            return;
//
//        f1 << "-- The pose information used by remapping." << std::endl;
//        f1 << "{" << std::endl;
//        f1 << "    to_trajectory_id = 0," << std::endl;
//        f1 << "    relative_pose = {" << std::endl;
//        f1 << "        translation = {" << std::endl;
//        f1 << "\t\t" << temp_translation_x << "," << std::endl;
//        f1 << "\t\t" << temp_translation_y << "," << std::endl;
//        f1 << "\t\t" << temp_translation_z << "," << std::endl;
//        f1 << "        }," << std::endl;
//        f1 << "        rotation = {" << std::endl;
//        f1 << "\t\tw=" << temp_rotation_w << ","<< std::endl;
//        f1 << "\t\tx=" << temp_rotation_x << ","<< std::endl;
//        f1 << "\t\ty=" << temp_rotation_y << ","<< std::endl;
//        f1 << "\t\tz=" << temp_rotation_z << ","<< std::endl;
//        f1 << "        }" << std::endl;
//        f1 << "    }" << std::endl;
//        f1 << "}" << std::endl;
//
//        f1.close();
//    }
//}

void Run() {
  constexpr double kTfBufferCacheTimeInSeconds = 10.;
  tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
  tf2_ros::TransformListener tf(tf_buffer);
  NodeOptions node_options;
  TrajectoryOptions trajectory_options;
  std::tie(node_options, trajectory_options) =
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

  auto map_builder = absl::make_unique<cartographer::mapping::MapBuilder>(
      node_options.map_builder_options);
  Node node(node_options, std::move(map_builder), &tf_buffer,
            FLAGS_collect_metrics);
  if (!FLAGS_load_state_filename.empty()) {
    node.LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
  }

  if (FLAGS_start_trajectory_with_default_topics) {
    node.StartTrajectoryWithDefaultTopics(trajectory_options);
  }

  ::ros::NodeHandle nh; // add
  ros::Subscriber sub_tf = nh.subscribe<tf2_msgs::TFMessage>("tf", 100, SaveToLua); // add

  ::ros::spin();

  node.FinishAllTrajectories();
  node.RunFinalOptimization();

  if (!FLAGS_save_state_filename.empty()) {
    node.SerializeState(FLAGS_save_state_filename,
                        true /* include_unfinished_submaps */);
  }
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  ::ros::init(argc, argv, "cartographer_node");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  cartographer_ros::Run();
  ::ros::shutdown();
}
