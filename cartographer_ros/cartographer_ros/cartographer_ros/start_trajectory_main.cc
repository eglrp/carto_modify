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

#include <string>
#include <vector>

#include "absl/memory/memory.h"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/trajectory_options.h"
#include "cartographer_ros_msgs/StartTrajectory.h"
#include "cartographer_ros_msgs/StatusCode.h"
#include "cartographer_ros_msgs/TrajectoryOptions.h"
#include "gflags/gflags.h"
#include "ros/ros.h"

// ed: header added
#include <chrono>
#include <thread>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
//#include <tf2/utils.h>
//#include <geometry_msgs/Transform.h>

#include "cartographer/io/proto_stream.h"
#include "cartographer_ros/submap.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(initial_pose, "", "Starting pose of a new trajectory");
// ed: pbstream filename to get last trajectory's position(x,y,z) value
DEFINE_string(pbstream_filename, "",
              "Filename of a pbstream to draw a map from.");

tf::Transform EOT_tf;

namespace cartographer_ros {
namespace {

TrajectoryOptions LoadOptions() {
  // 解析lua配置文件
  auto file_resolver =
      absl::make_unique<cartographer::common::ConfigurationFileResolver>(
          std::vector<std::string>{FLAGS_configuration_directory});
  const std::string code =
      file_resolver->GetFileContentOrDie(FLAGS_configuration_basename);
  auto lua_parameter_dictionary =
      cartographer::common::LuaParameterDictionary::NonReferenceCounted(
          code, std::move(file_resolver));
  // 解析init参数
  if (!FLAGS_initial_pose.empty()) {
    auto initial_trajectory_pose_file_resolver =
        absl::make_unique<cartographer::common::ConfigurationFileResolver>(
            std::vector<std::string>{FLAGS_configuration_directory});
    // 解析成::cartographer::common::LuaParameterDictionary类型的init pose
    auto initial_trajectory_pose =
        cartographer::common::LuaParameterDictionary::NonReferenceCounted(
            "return " + FLAGS_initial_pose,
            std::move(initial_trajectory_pose_file_resolver));
    // 调用trajectory_options.cc中的CreateTrajectoryOptions（双参数）
    return CreateTrajectoryOptions(lua_parameter_dictionary.get(),
                                   initial_trajectory_pose.get());
  } else {
    // 调用trajectory_options.cc中的CreateTrajectoryOptions（单参数）
    return CreateTrajectoryOptions(lua_parameter_dictionary.get());
  }
}

bool Run() {
  ros::NodeHandle node_handle;
  // 调用node.cc中的“kStartTrajectoryServiceName”服务
  ros::ServiceClient client_start =
      node_handle.serviceClient<cartographer_ros_msgs::StartTrajectory>(
          kStartTrajectoryServiceName);
  // 调用该服务的处理函数HandleStartTrajectory
  cartographer_ros_msgs::StartTrajectory srv_start;
  // 给::cartographer_ros_msgs::StartTrajectory::Request赋值
  // 重点是LoadOptions()
  srv_start.request.options = ToRosMessage(LoadOptions());
  srv_start.request.topics.laser_scan_topic = node_handle.resolveName(
      kLaserScanTopic, true /* apply topic remapping */);
  srv_start.request.topics.multi_echo_laser_scan_topic =
      node_handle.resolveName(kMultiEchoLaserScanTopic, true);
  srv_start.request.topics.point_cloud2_topic =
      node_handle.resolveName(kPointCloud2Topic, true);
  srv_start.request.topics.imu_topic = node_handle.resolveName(kImuTopic, true);
  srv_start.request.topics.odometry_topic =
      node_handle.resolveName(kOdometryTopic, true);
  // .call的返回类型为call(service.request, service.response, st::md5sum(service));
  if (!client_start.call(srv_start)) {
    LOG(ERROR) << "Failed to call " << kStartTrajectoryServiceName << ".";
    return false;
  }
  if (srv_start.response.status.code != cartographer_ros_msgs::StatusCode::OK) {
    LOG(ERROR) << "Error starting trajectory - message: '"
               << srv_start.response.status.message
               << "' (status code: " << std::to_string(srv_start.response.status.code)
               << ").";
    return false;
  }
  LOG(INFO) << "Started trajectory " << srv_start.response.trajectory_id;
  return true;
}

}  // namespace
}  // namespace cartographer_ros

void init_pose_callback(const geometry_msgs::Transform::ConstPtr& msg){

    std::cout << "[+] CB : /init_pose_string data received! " << std::endl;

//    ::cartographer::io::ProtoStreamReader reader(FLAGS_pbstream_filename);
//    cartographer_ros::NodeOptions node_options;
//    cartographer_ros::TrajectoryOptions trajectory_options;
//    std::tie(node_options, trajectory_options) =
//            cartographer_ros::LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);
//    auto map_builder = absl::make_unique<cartographer::mapping::MapBuilder>(
//            node_options.map_builder_options);
//
//    // ed: load pbstream
//    map_builder->LoadState(&reader, true);
//
//    // ed: get TrajectoryNodePose
//    const auto node_poses = map_builder->pose_graph()->GetTrajectoryNodePoses();
//    double pbstream_x = node_poses.BeginOfTrajectory(0)->data.global_pose.translation().x();
//    double pbstream_y = node_poses.BeginOfTrajectory(0)->data.global_pose.translation().y();
//    double pbstream_z = node_poses.BeginOfTrajectory(0)->data.global_pose.translation().z();
//    double pbstream_quat_x = node_poses.BeginOfTrajectory(0)->data.global_pose.rotation().x();
//    double pbstream_quat_y = node_poses.BeginOfTrajectory(0)->data.global_pose.rotation().y();
//    double pbstream_quat_z = node_poses.BeginOfTrajectory(0)->data.global_pose.rotation().z();
//    double pbstream_quat_w = node_poses.BeginOfTrajectory(0)->data.global_pose.rotation().w();

    tf::Transform map_tf;
//    tf::Transform EOT_tf;
    tf::Transform output_tf;

//    EOT_tf.setOrigin(tf::Vector3(pbstream_x,
//                                 pbstream_y,
//                                 pbstream_z));
//    EOT_tf.setRotation(tf::Quaternion(pbstream_quat_x,
//                                      pbstream_quat_y,
//                                      pbstream_quat_z,
//                                      pbstream_quat_w));

    map_tf.setOrigin(tf::Vector3(msg->translation.x,
                                 msg->translation.y,
                                 msg->translation.z));
    map_tf.setRotation(tf::Quaternion(msg->rotation.x,
                                      msg->rotation.y,
                                      msg->rotation.z,
                                      msg->rotation.w));

//    output_tf = EOT_tf * map_tf;
    output_tf = map_tf;

    // ed: Quaternion to RPY
    tf::Quaternion q(
            output_tf.getRotation().x(),
            output_tf.getRotation().y(),
            output_tf.getRotation().z(),
            output_tf.getRotation().w());
    tf::Matrix3x3 m(q); // 四元数转为旋转矩阵

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw); // 旋转矩阵转为欧拉角

    int to_trajectory_id = 0;   // ed: offline map's trajectory

    std::cout << "[+] map_tf : ["
              << map_tf.getOrigin().x() << ", "
              << map_tf.getOrigin().y() << ", "
              << map_tf.getOrigin().z() << "] " << std::endl;

    std::cout << "[+] EOT_tf : ["
              << EOT_tf.getOrigin().x() << ", "
              << EOT_tf.getOrigin().y() << ", "
              << EOT_tf.getOrigin().z() << "] " << std::endl;

    std::cout << "[+] output_tf : ["
              << output_tf.getOrigin().x() << ", "
              << output_tf.getOrigin().y() << ", "
              << output_tf.getOrigin().z() << "] " << std::endl;

    // ed: pitch isn't used
    std::string parsed_string ="{to_trajectory_id = "    \
                             + std::to_string(to_trajectory_id)         \
                             + ", relative_pose = { translation = { "   \
                             + std::to_string(output_tf.getOrigin().x()) + ", " \
                             + std::to_string(output_tf.getOrigin().y()) + ", " \
                             + std::to_string(output_tf.getOrigin().z()) + \
                             + "}, rotation = { "\
                             + std::to_string(roll) + ","       \
                             + std::to_string(pitch) + ","      \
                             + std::to_string(yaw) + "}}}";

    // ed: insert value into FLAGS_initial_pose string
    FLAGS_initial_pose = parsed_string;

}

void initialpose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg){

    std::cout << "[+] CB : /initialpose data received! " << std::endl;

//    ::cartographer::io::ProtoStreamReader reader(FLAGS_pbstream_filename);
//    cartographer_ros::NodeOptions node_options;
//    cartographer_ros::TrajectoryOptions trajectory_options;
//    std::tie(node_options, trajectory_options) =
//            cartographer_ros::LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);
//    auto map_builder = absl::make_unique<cartographer::mapping::MapBuilder>(
//            node_options.map_builder_options);
//
//    // ed: load pbstream
//    map_builder->LoadState(&reader, true);
//
//    // ed: get TrajectoryNodePose
//    const auto node_poses = map_builder->pose_graph()->GetTrajectoryNodePoses();
//    double pbstream_x = node_poses.BeginOfTrajectory(0)->data.global_pose.translation().x();
//    double pbstream_y = node_poses.BeginOfTrajectory(0)->data.global_pose.translation().y();
//    double pbstream_z = node_poses.BeginOfTrajectory(0)->data.global_pose.translation().z();
//    double pbstream_quat_x = node_poses.BeginOfTrajectory(0)->data.global_pose.rotation().x();
//    double pbstream_quat_y = node_poses.BeginOfTrajectory(0)->data.global_pose.rotation().y();
//    double pbstream_quat_z = node_poses.BeginOfTrajectory(0)->data.global_pose.rotation().z();
//    double pbstream_quat_w = node_poses.BeginOfTrajectory(0)->data.global_pose.rotation().w();

    tf::Transform map_tf;
//    tf::Transform EOT_tf;
    tf::Transform output_tf;

//    EOT_tf.setOrigin(tf::Vector3(pbstream_x,
//                                 pbstream_y,
//                                 pbstream_z));
//    EOT_tf.setRotation(tf::Quaternion(pbstream_quat_x,
//                                      pbstream_quat_y,
//                                      pbstream_quat_z,
//                                      pbstream_quat_w));

    map_tf.setOrigin(tf::Vector3(msg->pose.pose.position.x,
                                 msg->pose.pose.position.y,
                                 msg->pose.pose.position.z));
    map_tf.setRotation(tf::Quaternion(msg->pose.pose.orientation.x,
                                      msg->pose.pose.orientation.y,
                                      msg->pose.pose.orientation.z,
                                      msg->pose.pose.orientation.w));

//    output_tf = EOT_tf * map_tf;
    output_tf = map_tf;

    // ed: Quaternion to RPY
    tf::Quaternion q(
            output_tf.getRotation().x(),
            output_tf.getRotation().y(),
            output_tf.getRotation().z(),
            output_tf.getRotation().w());
    tf::Matrix3x3 m(q); // 四元数转为旋转矩阵

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw); // 旋转矩阵转为欧拉角

    int to_trajectory_id = 0;   // ed: offline map's trajectory

    std::cout << "[+] map_tf : ["
              << map_tf.getOrigin().x() << ", "
              << map_tf.getOrigin().y() << ", "
              << map_tf.getOrigin().z() << "] " << std::endl;

    std::cout << "[+] EOT_tf : ["
              << EOT_tf.getOrigin().x() << ", "
              << EOT_tf.getOrigin().y() << ", "
              << EOT_tf.getOrigin().z() << "] " << std::endl;

    std::cout << "[+] output_tf : ["
              << output_tf.getOrigin().x() << ", "
              << output_tf.getOrigin().y() << ", "
              << output_tf.getOrigin().z() << "] " << std::endl;

    // ed: pitch isn't used
    std::string parsed_string ="{to_trajectory_id = "    \
                             + std::to_string(to_trajectory_id)         \
                             + ", relative_pose = { translation = { "   \
                             + std::to_string(output_tf.getOrigin().x()) + ", " \
                             + std::to_string(output_tf.getOrigin().y()) + ", " \
                             + std::to_string(output_tf.getOrigin().z()) + \
                             + "}, rotation = { "\
                             + std::to_string(roll) + ","       \
                             + std::to_string(pitch) + ","      \
                             + std::to_string(yaw) + "}}}";

    // ed: insert value into FLAGS_initial_pose string
    FLAGS_initial_pose = parsed_string;

}

// ed: /move_base_simple/goal (2D Nav Goal in Rviz) subscribe callback function
//void move_base_simple_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
//  std::cout << "[+] CB : /move_base_simple/goal data received! " << std::endl;
//
//  ::cartographer::io::ProtoStreamReader reader(FLAGS_pbstream_filename);
//  cartographer_ros::NodeOptions node_options;
//  cartographer_ros::TrajectoryOptions trajectory_options;
//  std::tie(node_options, trajectory_options) =
//          cartographer_ros::LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);
//  auto map_builder = absl::make_unique<cartographer::mapping::MapBuilder>(
//                  node_options.map_builder_options);
//
//  // ed: load pbstream
//  map_builder->LoadState(&reader, true);
//
//  // ed: get TrajectoryNodePose
//  const auto node_poses = map_builder->pose_graph()->GetTrajectoryNodePoses();
//  double pbstream_x = std::prev(node_poses.EndOfTrajectory(0))->data.global_pose.translation().x();
//  double pbstream_y = std::prev(node_poses.EndOfTrajectory(0))->data.global_pose.translation().y();
//  double pbstream_z = std::prev(node_poses.EndOfTrajectory(0))->data.global_pose.translation().z();
//  double pbstream_quat_x = (std::prev(node_poses.EndOfTrajectory(0)))->data.global_pose.rotation().x();
//  double pbstream_quat_y = (std::prev(node_poses.EndOfTrajectory(0)))->data.global_pose.rotation().y();
//  double pbstream_quat_z = (std::prev(node_poses.EndOfTrajectory(0)))->data.global_pose.rotation().z();
//  double pbstream_quat_w = (std::prev(node_poses.EndOfTrajectory(0)))->data.global_pose.rotation().w();
//
//  tf::Transform map_tf;
//  tf::Transform EOT_tf;
//  tf::Transform output_tf;
//
//  map_tf.setOrigin(tf::Vector3(msg->pose.position.x,
//                               msg->pose.position.y,
//                               msg->pose.position.z));
//  map_tf.setRotation(tf::Quaternion(msg->pose.orientation.x,
//                                    msg->pose.orientation.y,
//                                    msg->pose.orientation.z,
//                                    msg->pose.orientation.w));
//
//  EOT_tf.setOrigin(tf::Vector3(pbstream_x,
//                               pbstream_y,
//                               pbstream_z));
//  EOT_tf.setRotation(tf::Quaternion(pbstream_quat_x,
//                                    pbstream_quat_y,
//                                    pbstream_quat_z,
//                                    pbstream_quat_w));
//
////  output_tf = EOT_tf * map_tf;
//  output_tf = map_tf;
//
//
//    // ed: Quaternion to RPY
//  tf::Quaternion q(
//          output_tf.getRotation().x(),
//          output_tf.getRotation().y(),
//          output_tf.getRotation().z(),
//          output_tf.getRotation().w());
//  tf::Matrix3x3 m(q);
//
//  double roll, pitch, yaw;
//  m.getRPY(roll, pitch, yaw);
//
//  int to_trajectory_id = 0;   // ed: offline map's trajectory
//
//  std::cout << "[+] map_tf : ["
//            << map_tf.getOrigin().x() << ", "
//            << map_tf.getOrigin().y() << ", "
//            << map_tf.getOrigin().z() << "] " << std::endl;
//
//  std::cout << "[+] EOT_tf : ["
//            << EOT_tf.getOrigin().x() << ", "
//            << EOT_tf.getOrigin().y() << ", "
//            << EOT_tf.getOrigin().z() << "] " << std::endl;
//
//  std::cout << "[+] output_tf : ["
//            << output_tf.getOrigin().x() << ", "
//            << output_tf.getOrigin().y() << ", "
//            << output_tf.getOrigin().z() << "] " << std::endl;
//
//  // ed: pitch isn't used
//  std::string parsed_string ="{to_trajectory_id = "    \
//                             + std::to_string(to_trajectory_id)         \
//                             + ", relative_pose = { translation = { "   \
//                             + std::to_string(output_tf.getOrigin().x()) + ", " \
//                             + std::to_string(output_tf.getOrigin().y()) + ", " \
//                             + "0" + \
//                             + "}, rotation = { "\
//                             + std::to_string(roll) + ","       \
//                             + std::to_string(0) + ","      \
//                             + std::to_string(yaw) + "}}}";
//
//
//  // ed: insert value into FLAGS_initial_pose string
//  FLAGS_initial_pose = parsed_string;
//}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::SetUsageMessage(
      "\n\n"
      "Convenience tool around the start_trajectory service. This takes a Lua "
      "file that is accepted by the node as well and starts a new trajectory "
      "using its settings.\n");
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  ::ros::init(argc, argv, "cartographer_start_trajectory");
  ::ros::start();

  // ed: code added
  ::ros::NodeHandle nh;
  ::ros::Subscriber sub_rviz_init = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1,
                                                                                           &initialpose_callback);
  ::ros::Subscriber sub_init_pose = nh.subscribe<geometry_msgs::Transform>("/init_pose", 1, ::init_pose_callback);
//  ::ros::Subscriber sub_init_pose = nh.subscribe<std_msgs::String>("/init_pose_string", 1, ::init_pose_callback);
//  ::ros::Subscriber sub_move_base_simple = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &move_base_simple_callback);
  ::ros::Rate loop_rate(1);

  // ed: while loop until /init_pose_string
  while(::ros::ok()) {
    std::cout << "waiting for data..." << std::endl;

    if(!FLAGS_initial_pose.empty()){
      sub_init_pose.shutdown();

      std::cout << "data received : " << FLAGS_initial_pose << std::endl;

      cartographer_ros::ScopedRosLogSink ros_log_sink;
      int exit_code = cartographer_ros::Run() ? 0 : 1;

      ::ros::shutdown();
      return exit_code;
    }

    ::ros::spinOnce();
    loop_rate.sleep();
  }
}
