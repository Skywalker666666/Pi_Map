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

#include "cartographer_ros/node.h"

#include <chrono>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/time_conversion.h"
#include "cartographer_ros_msgs/StatusCode.h"
#include "cartographer_ros_msgs/StatusResponse.h"
#include "glog/logging.h"
#include "nav_msgs/Odometry.h"
#include "ros/serialization.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_eigen/tf2_eigen.h"
#include "visualization_msgs/MarkerArray.h"

#include <stdio.h>
#include <functional>
#include <iostream>
#include <thread>

#include <nanomsg/nn.h>
#include <nanomsg/reqrep.h>
#include <nanomsg/pubsub.h>

#include <ctime>
#include <fstream>
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer_ros/ros_map.h"
#include "cartographer_ros/submap.h"

namespace cartographer_ros {

namespace {

cartographer_ros_msgs::SensorTopics DefaultSensorTopics() {
  cartographer_ros_msgs::SensorTopics topics;
  topics.laser_scan_topic = kLaserScanTopic;
  topics.multi_echo_laser_scan_topic = kMultiEchoLaserScanTopic;
  topics.point_cloud2_topic = kPointCloud2Topic;
  topics.imu_topic = kImuTopic;
  topics.odometry_topic = kOdometryTopic;
  topics.nav_sat_fix_topic = kNavSatFixTopic;
  topics.landmark_topic = kLandmarkTopic;
  return topics;
}

// Subscribes to the 'topic' for 'trajectory_id' using the 'node_handle' and
// calls 'handler' on the 'node' to handle messages. Returns the subscriber.
template <typename MessageType>
::ros::Subscriber SubscribeWithHandler(
    void (Node::*handler)(int, const std::string&,
                          const typename MessageType::ConstPtr&),
    const int trajectory_id, const std::string& topic,
    ::ros::NodeHandle* const node_handle, Node* const node) {
  return node_handle->subscribe<MessageType>(
      topic, kInfiniteSubscriberQueueSize,
      boost::function<void(const typename MessageType::ConstPtr&)>(
          [node, handler, trajectory_id,
           topic](const typename MessageType::ConstPtr& msg) {
            (node->*handler)(trajectory_id, topic, msg);
          }));
}

}  // namespace

namespace carto = ::cartographer;

using carto::transform::Rigid3d;

using namespace std;

//For pose send frequency calculation.
//std::chrono::high_resolution_clock::time_point pose_time3 = {};
//std::chrono::high_resolution_clock::time_point pose_time3 = std::chrono::high_resolution_clock::time_point::min();
std::chrono::high_resolution_clock::time_point pose_time3 = std::chrono::high_resolution_clock::now();


Node::Node(
    const NodeOptions& node_options,
    std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
    tf2_ros::Buffer* const tf_buffer)
    : node_options_(node_options),
      map_builder_bridge_(node_options_, std::move(map_builder), tf_buffer) {
  carto::common::MutexLocker lock(&mutex_);
  submap_list_publisher_ =
      node_handle_.advertise<::cartographer_ros_msgs::SubmapList>(
          kSubmapListTopic, kLatestOnlyPublisherQueueSize);
  trajectory_node_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          kTrajectoryNodeListTopic, kLatestOnlyPublisherQueueSize);
  landmark_poses_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          kLandmarkPosesListTopic, kLatestOnlyPublisherQueueSize);
  constraint_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          kConstraintListTopic, kLatestOnlyPublisherQueueSize);
  service_servers_.push_back(node_handle_.advertiseService(
      kSubmapQueryServiceName, &Node::HandleSubmapQuery, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kStartTrajectoryServiceName, &Node::HandleStartTrajectory, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kFinishTrajectoryServiceName, &Node::HandleFinishTrajectory, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kWriteStateServiceName, &Node::HandleWriteState, this));

  scan_matched_point_cloud_publisher_ =
      node_handle_.advertise<sensor_msgs::PointCloud2>(
          kScanMatchedPointCloudTopic, kLatestOnlyPublisherQueueSize);

  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.submap_publish_period_sec),
      &Node::PublishSubmapList, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.pose_publish_period_sec),
      &Node::PublishTrajectoryStates, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.trajectory_publish_period_sec),
      &Node::PublishTrajectoryNodeList, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.trajectory_publish_period_sec),
      &Node::PublishLandmarkPosesList, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(kConstraintPublishPeriodSec),
      &Node::PublishConstraintList, this));
  
    //char url[64] = "ipc:///home/julian/test_reqrep.ipc";
  char url[64] = "ipc:///tmp/pos_data.ipc";

  msg_adp = std::make_shared<PIAUTO::msg::PIMsgAdaptor>();

  pub_handler = msg_adp->registerOneMessageChannel({NN_PUB, url, "robot_pos", -1});
  
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

Node::~Node() { FinishAllTrajectories(); }

::ros::NodeHandle* Node::node_handle() { return &node_handle_; }

bool Node::HandleSubmapQuery(
    ::cartographer_ros_msgs::SubmapQuery::Request& request,
    ::cartographer_ros_msgs::SubmapQuery::Response& response) {
  carto::common::MutexLocker lock(&mutex_);
  map_builder_bridge_.HandleSubmapQuery(request, response);
  return true;
}

void Node::PublishSubmapList(const ::ros::WallTimerEvent& unused_timer_event) {
  carto::common::MutexLocker lock(&mutex_);
  submap_list_publisher_.publish(map_builder_bridge_.GetSubmapList());
}

void Node::AddExtrapolator(const int trajectory_id,
                           const TrajectoryOptions& options) {
  constexpr double kExtrapolationEstimationTimeSec = 0.001;  // 1 ms
  CHECK(extrapolators_.count(trajectory_id) == 0);
  const double gravity_time_constant =
      node_options_.map_builder_options.use_trajectory_builder_3d()
          ? options.trajectory_builder_options.trajectory_builder_3d_options()
                .imu_gravity_time_constant()
          : options.trajectory_builder_options.trajectory_builder_2d_options()
                .imu_gravity_time_constant();
  extrapolators_.emplace(
      std::piecewise_construct, std::forward_as_tuple(trajectory_id),
      std::forward_as_tuple(
          ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
          gravity_time_constant));
}

void Node::AddSensorSamplers(const int trajectory_id,
                             const TrajectoryOptions& options) {
  CHECK(sensor_samplers_.count(trajectory_id) == 0);
  sensor_samplers_.emplace(
      std::piecewise_construct, std::forward_as_tuple(trajectory_id),
      std::forward_as_tuple(
          options.rangefinder_sampling_ratio, options.odometry_sampling_ratio,
          options.fixed_frame_pose_sampling_ratio, options.imu_sampling_ratio,
          options.landmarks_sampling_ratio));
}

void Node::PublishTrajectoryStates(const ::ros::WallTimerEvent& timer_event) {
  carto::common::MutexLocker lock(&mutex_);
  for (const auto& entry : map_builder_bridge_.GetTrajectoryStates()) {
    const auto& trajectory_state = entry.second;

    auto& extrapolator = extrapolators_.at(entry.first);
    // We only publish a point cloud if it has changed. It is not needed at high
    // frequency, and republishing it would be computationally wasteful.
    
    if (trajectory_state.local_slam_data->time !=
        extrapolator.GetLastPoseTime()) {
      
    //if (node_id_in_local_ !=
    //    map_builder_bridge_.GetNodeId(0)) {      

      //added by skywalker
      node_id_in_local_ = map_builder_bridge_.GetNodeId(0);      
    
      if (scan_matched_point_cloud_publisher_.getNumSubscribers() > 0) {
        // TODO(gaschler): Consider using other message without time
        // information.
        carto::sensor::TimedPointCloud point_cloud;
        point_cloud.reserve(trajectory_state.local_slam_data
                                ->range_data_in_local.returns.size());
        for (const Eigen::Vector3f point :
             trajectory_state.local_slam_data->range_data_in_local.returns) {
          Eigen::Vector4f point_time;
          point_time << point, 0.f;
          point_cloud.push_back(point_time);
        }
        scan_matched_point_cloud_publisher_.publish(ToPointCloud2Message(
            carto::common::ToUniversal(trajectory_state.local_slam_data->time),
            node_options_.map_frame,
            carto::sensor::TransformTimedPointCloud(
                point_cloud, trajectory_state.local_to_map.cast<float>())));
      }
      extrapolator.AddPose(trajectory_state.local_slam_data->time,
                           trajectory_state.local_slam_data->local_pose);
      
      // for post processing of tf interpolate 
      pose_publisher_tx_counter_ += 1;
    }

    geometry_msgs::TransformStamped stamped_transform;
    // If we do not publish a new point cloud, we still allow time of the
    // published poses to advance. If we already know a newer pose, we use its
    // time instead. Since tf knows how to interpolate, providing newer
    // information is better.
    const ::cartographer::common::Time now = std::max(
        FromRos(ros::Time::now()), extrapolator.GetLastExtrapolatedTime());
    stamped_transform.header.stamp = ToRos(now);

    const Rigid3d tracking_to_local = [&] {
      if (trajectory_state.trajectory_options.publish_frame_projected_to_2d) {
        return carto::transform::Embed3D(
            carto::transform::Project2D(extrapolator.ExtrapolatePose(now)));
      }
      return extrapolator.ExtrapolatePose(now);
    }();
    //LOG(INFO) << "tracking_to_local: " << tracking_to_local;  // added by Skywalker
    //LOG(INFO) << "It is supposed to be local pose, but there is global map information.";

    const Rigid3d tracking_to_map =
        trajectory_state.local_to_map * tracking_to_local;

    // added by Skywalker
    const ::cartographer::common::Time now2 = FromRos(ros::Time::now());
    
	

    if (trajectory_state.published_to_tracking != nullptr) {
      if (trajectory_state.trajectory_options.provide_odom_frame) {
        std::vector<geometry_msgs::TransformStamped> stamped_transforms;

        stamped_transform.header.frame_id = node_options_.map_frame;
        stamped_transform.child_frame_id =
            trajectory_state.trajectory_options.odom_frame;
        stamped_transform.transform =
            ToGeometryMsgTransform(trajectory_state.local_to_map);
        stamped_transforms.push_back(stamped_transform);

        stamped_transform.header.frame_id =
            trajectory_state.trajectory_options.odom_frame;
        stamped_transform.child_frame_id =
            trajectory_state.trajectory_options.published_frame;
        stamped_transform.transform = ToGeometryMsgTransform(
            tracking_to_local * (*trajectory_state.published_to_tracking));
        stamped_transforms.push_back(stamped_transform);

        //comment from here	
        pose_tx_counter_ += 1;
        
        original_local_poses_.push_back({
                  pose_tx_counter_,
                  trajectory_state.local_slam_data->time,
                  node_id_in_local_,
                  trajectory_state.local_slam_data->local_pose,
          	  pose_publisher_tx_counter_
                  });
        
        optimized_global_poses_.push_back({
	        pose_tx_counter_,
	        trajectory_state.local_slam_data->time,
	        node_id_in_local_,
	        trajectory_state.local_to_map * trajectory_state.local_slam_data->local_pose,
		pose_publisher_tx_counter_
                });
      
        loctomap_adjust_poses_.push_back({
                  pose_tx_counter_,
                  trajectory_state.local_slam_data->time,
          	node_id_in_local_,
                  trajectory_state.local_to_map,
                  pose_publisher_tx_counter_
                  });      
        
        
        int tUnitSize = sizeof(stamped_transform.transform.translation.x);
        int rUnitSize = sizeof(stamped_transform.transform.rotation.x);
        int idUnitSize = sizeof(pose_tx_counter_);
        int timestpUnitSize = sizeof(trajectory_state.local_slam_data->time);//time stamp size
        
        int bufSize = tUnitSize * 3 + rUnitSize * 4 + idUnitSize * 2 + timestpUnitSize;
        char sendBuf[bufSize];
        memcpy(sendBuf, &(pose_tx_counter_), idUnitSize);
        memcpy(sendBuf + idUnitSize, &(node_id_in_local_), idUnitSize);
        memcpy(sendBuf + idUnitSize * 2, &(trajectory_state.local_slam_data->time), timestpUnitSize);   
        memcpy(sendBuf + idUnitSize * 2 + timestpUnitSize, &(stamped_transform.transform.translation.x), tUnitSize);
        memcpy(sendBuf + idUnitSize * 2 + timestpUnitSize + tUnitSize, &(stamped_transform.transform.translation.y), tUnitSize);
        memcpy(sendBuf + idUnitSize * 2 + timestpUnitSize + tUnitSize * 2, &(stamped_transform.transform.translation.z), tUnitSize);
        memcpy(sendBuf + idUnitSize * 2 + timestpUnitSize + tUnitSize * 3, &(stamped_transform.transform.rotation.w), rUnitSize);
        memcpy(sendBuf + idUnitSize * 2 + timestpUnitSize + tUnitSize * 3 + rUnitSize, &(stamped_transform.transform.rotation.x), rUnitSize);
        memcpy(sendBuf + idUnitSize * 2 + timestpUnitSize + tUnitSize * 3 + rUnitSize * 2, &(stamped_transform.transform.rotation.y), rUnitSize);
        memcpy(sendBuf + idUnitSize * 2 + timestpUnitSize + tUnitSize * 3 + rUnitSize * 3, &(stamped_transform.transform.rotation.z), rUnitSize);
  	
        pi_msg_envelope_t env;
	memcpy(env.filter, "robot_pos", sizeof("robot_pos"));
	env.type = PIMSG_ROBOT_POS_2D;
        env.length = bufSize;

        int rc = msg_adp->send(pub_handler, &env, sendBuf, env.length, nullptr);
        //comment end here 
        
        tf_broadcaster_.sendTransform(stamped_transforms);

      } else {
        stamped_transform.header.frame_id = node_options_.map_frame;
        stamped_transform.child_frame_id =
            trajectory_state.trajectory_options.published_frame;
        stamped_transform.transform = ToGeometryMsgTransform(
            tracking_to_map * (*trajectory_state.published_to_tracking));

        tf_broadcaster_.sendTransform(stamped_transform);
      }
    }
  }
}

void Node::PublishTrajectoryNodeList(
    const ::ros::WallTimerEvent& unused_timer_event) {
  if (trajectory_node_list_publisher_.getNumSubscribers() > 0) {
    carto::common::MutexLocker lock(&mutex_);
    trajectory_node_list_publisher_.publish(
        map_builder_bridge_.GetTrajectoryNodeList());
  }
}

void Node::PublishLandmarkPosesList(
    const ::ros::WallTimerEvent& unused_timer_event) {
  if (landmark_poses_list_publisher_.getNumSubscribers() > 0) {
    carto::common::MutexLocker lock(&mutex_);
    landmark_poses_list_publisher_.publish(
        map_builder_bridge_.GetLandmarkPosesList());
  }
}

void Node::PublishConstraintList(
    const ::ros::WallTimerEvent& unused_timer_event) {
  if (constraint_list_publisher_.getNumSubscribers() > 0) {
    carto::common::MutexLocker lock(&mutex_);
    constraint_list_publisher_.publish(map_builder_bridge_.GetConstraintList());
  }
}

std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
Node::ComputeExpectedSensorIds(
    const TrajectoryOptions& options,
    const cartographer_ros_msgs::SensorTopics& topics) const {
  using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
  using SensorType = SensorId::SensorType;
  std::set<SensorId> expected_topics;
  // Subscribe to all laser scan, multi echo laser scan, and point cloud topics.
  for (const std::string& topic : ComputeRepeatedTopicNames(
           topics.laser_scan_topic, options.num_laser_scans)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  for (const std::string& topic :
       ComputeRepeatedTopicNames(topics.multi_echo_laser_scan_topic,
                                 options.num_multi_echo_laser_scans)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  for (const std::string& topic : ComputeRepeatedTopicNames(
           topics.point_cloud2_topic, options.num_point_clouds)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    expected_topics.insert(SensorId{SensorType::IMU, topics.imu_topic});
  }
  // Odometry is optional.
  if (options.use_odometry) {
    expected_topics.insert(
        SensorId{SensorType::ODOMETRY, topics.odometry_topic});
  }
  // NavSatFix is optional.
  if (options.use_nav_sat) {
    expected_topics.insert(
        SensorId{SensorType::FIXED_FRAME_POSE, topics.nav_sat_fix_topic});
  }
  // Landmark is optional.
  if (options.use_landmarks) {
    expected_topics.insert(SensorId{SensorType::LANDMARK, kLandmarkTopic});
  }
  return expected_topics;
}

int Node::AddTrajectory(const TrajectoryOptions& options,
                        const cartographer_ros_msgs::SensorTopics& topics) {
  const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
      expected_sensor_ids = ComputeExpectedSensorIds(options, topics);
  const int trajectory_id =
      map_builder_bridge_.AddTrajectory(expected_sensor_ids, options);
  AddExtrapolator(trajectory_id, options);
  AddSensorSamplers(trajectory_id, options);
  LaunchSubscribers(options, topics, trajectory_id);
  is_active_trajectory_[trajectory_id] = true;
  for (const auto& sensor_id : expected_sensor_ids) {
    subscribed_topics_.insert(sensor_id.id);
  }
  return trajectory_id;
}

void Node::LaunchSubscribers(const TrajectoryOptions& options,
                             const cartographer_ros_msgs::SensorTopics& topics,
                             const int trajectory_id) {
  for (const std::string& topic : ComputeRepeatedTopicNames(
           topics.laser_scan_topic, options.num_laser_scans)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::LaserScan>(
             &Node::HandleLaserScanMessage, trajectory_id, topic, &node_handle_,
             this),
         topic});
  }
  for (const std::string& topic :
       ComputeRepeatedTopicNames(topics.multi_echo_laser_scan_topic,
                                 options.num_multi_echo_laser_scans)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::MultiEchoLaserScan>(
             &Node::HandleMultiEchoLaserScanMessage, trajectory_id, topic,
             &node_handle_, this),
         topic});
  }
  for (const std::string& topic : ComputeRepeatedTopicNames(
           topics.point_cloud2_topic, options.num_point_clouds)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::PointCloud2>(
             &Node::HandlePointCloud2Message, trajectory_id, topic,
             &node_handle_, this),
         topic});
  }

  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    std::string topic = topics.imu_topic;
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::Imu>(&Node::HandleImuMessage,
                                                trajectory_id, topic,
                                                &node_handle_, this),
         topic});
  }

  if (options.use_odometry) {
    std::string topic = topics.odometry_topic;
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<nav_msgs::Odometry>(&Node::HandleOdometryMessage,
                                                  trajectory_id, topic,
                                                  &node_handle_, this),
         topic});
  }
  if (options.use_nav_sat) {
    std::string topic = topics.nav_sat_fix_topic;
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::NavSatFix>(
             &Node::HandleNavSatFixMessage, trajectory_id, topic, &node_handle_,
             this),
         topic});
  }
  if (options.use_landmarks) {
    std::string topic = topics.landmark_topic;
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<cartographer_ros_msgs::LandmarkList>(
             &Node::HandleLandmarkMessage, trajectory_id, topic, &node_handle_,
             this),
         topic});
  }
}

bool Node::ValidateTrajectoryOptions(const TrajectoryOptions& options) {
  if (node_options_.map_builder_options.use_trajectory_builder_2d()) {
    return options.trajectory_builder_options
        .has_trajectory_builder_2d_options();
  }
  if (node_options_.map_builder_options.use_trajectory_builder_3d()) {
    return options.trajectory_builder_options
        .has_trajectory_builder_3d_options();
  }
  return false;
}

bool Node::ValidateTopicNames(
    const ::cartographer_ros_msgs::SensorTopics& topics,
    const TrajectoryOptions& options) {
  for (const auto& sensor_id : ComputeExpectedSensorIds(options, topics)) {
    const std::string& topic = sensor_id.id;
    if (subscribed_topics_.count(topic) > 0) {
      LOG(ERROR) << "Topic name [" << topic << "] is already used.";
      return false;
    }
  }
  return true;
}

cartographer_ros_msgs::StatusResponse Node::FinishTrajectoryUnderLock(
    const int trajectory_id) {
  cartographer_ros_msgs::StatusResponse status_response;

  // First, check if we can actually finish the trajectory.
  if (map_builder_bridge_.GetFrozenTrajectoryIds().count(trajectory_id)) {
    const std::string error =
        "Trajectory " + std::to_string(trajectory_id) + " is frozen.";
    LOG(ERROR) << error;
    status_response.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
    status_response.message = error;
    return status_response;
  }
  if (is_active_trajectory_.count(trajectory_id) == 0) {
    const std::string error =
        "Trajectory " + std::to_string(trajectory_id) + " is not created yet.";
    LOG(ERROR) << error;
    status_response.code = cartographer_ros_msgs::StatusCode::NOT_FOUND;
    status_response.message = error;
    return status_response;
  }
  if (!is_active_trajectory_[trajectory_id]) {
    const std::string error = "Trajectory " + std::to_string(trajectory_id) +
                              " has already been finished.";
    LOG(ERROR) << error;
    status_response.code =
        cartographer_ros_msgs::StatusCode::RESOURCE_EXHAUSTED;
    status_response.message = error;
    return status_response;
  }

  // Shutdown the subscribers of this trajectory.
  for (auto& entry : subscribers_[trajectory_id]) {
    entry.subscriber.shutdown();
    subscribed_topics_.erase(entry.topic);
    LOG(INFO) << "Shutdown the subscriber of [" << entry.topic << "]";
  }
  CHECK_EQ(subscribers_.erase(trajectory_id), 1);
  CHECK(is_active_trajectory_.at(trajectory_id));
  map_builder_bridge_.FinishTrajectory(trajectory_id);
  is_active_trajectory_[trajectory_id] = false;
  const std::string message =
      "Finished trajectory " + std::to_string(trajectory_id) + ".";
  status_response.code = cartographer_ros_msgs::StatusCode::OK;
  status_response.message = message;
  return status_response;
}

bool Node::HandleStartTrajectory(
    ::cartographer_ros_msgs::StartTrajectory::Request& request,
    ::cartographer_ros_msgs::StartTrajectory::Response& response) {
  carto::common::MutexLocker lock(&mutex_);
  TrajectoryOptions options;
  if (!FromRosMessage(request.options, &options) ||
      !ValidateTrajectoryOptions(options)) {
    const std::string error = "Invalid trajectory options.";
    LOG(ERROR) << error;
    response.status.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
    response.status.message = error;
  } else if (!ValidateTopicNames(request.topics, options)) {
    const std::string error = "Invalid topics.";
    LOG(ERROR) << error;
    response.status.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
    response.status.message = error;
  } else {
    response.trajectory_id = AddTrajectory(options, request.topics);
    response.status.code = cartographer_ros_msgs::StatusCode::OK;
    response.status.message = "Success.";
  }
  return true;
}

void Node::StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options) {
  carto::common::MutexLocker lock(&mutex_);
  CHECK(ValidateTrajectoryOptions(options));
  AddTrajectory(options, DefaultSensorTopics());
}

std::vector<
    std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>>
Node::ComputeDefaultSensorIdsForMultipleBags(
    const std::vector<TrajectoryOptions>& bags_options) const {
  using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
  std::vector<std::set<SensorId>> bags_sensor_ids;
  for (size_t i = 0; i < bags_options.size(); ++i) {
    std::string prefix;
    if (bags_options.size() > 1) {
      prefix = "bag_" + std::to_string(i + 1) + "_";
    }
    std::set<SensorId> unique_sensor_ids;
    for (const auto& sensor_id :
         ComputeExpectedSensorIds(bags_options.at(i), DefaultSensorTopics())) {
      unique_sensor_ids.insert(SensorId{sensor_id.type, prefix + sensor_id.id});
    }
    bags_sensor_ids.push_back(unique_sensor_ids);
  }
  return bags_sensor_ids;
}

int Node::AddOfflineTrajectory(
    const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
        expected_sensor_ids,
    const TrajectoryOptions& options) {
  carto::common::MutexLocker lock(&mutex_);
  const int trajectory_id =
      map_builder_bridge_.AddTrajectory(expected_sensor_ids, options);
  AddExtrapolator(trajectory_id, options);
  AddSensorSamplers(trajectory_id, options);
  is_active_trajectory_[trajectory_id] = true;
  return trajectory_id;
}

bool Node::HandleFinishTrajectory(
    ::cartographer_ros_msgs::FinishTrajectory::Request& request,
    ::cartographer_ros_msgs::FinishTrajectory::Response& response) {
  carto::common::MutexLocker lock(&mutex_);
  response.status = FinishTrajectoryUnderLock(request.trajectory_id);
  return true;
}

bool Node::HandleWriteState(
    ::cartographer_ros_msgs::WriteState::Request& request,
    ::cartographer_ros_msgs::WriteState::Response& response) {
  carto::common::MutexLocker lock(&mutex_);
  if (map_builder_bridge_.SerializeState(request.filename)) {
    response.status.code = cartographer_ros_msgs::StatusCode::OK;
    response.status.message = "State written to '" + request.filename + "'.";
  } else {
    response.status.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
    response.status.message = "Failed to write '" + request.filename + "'.";
  }
  return true;
}

void Node::FinishAllTrajectories() {
  carto::common::MutexLocker lock(&mutex_);
  for (auto& entry : is_active_trajectory_) {
    const int trajectory_id = entry.first;
    if (entry.second) {
      CHECK_EQ(FinishTrajectoryUnderLock(trajectory_id).code,
               cartographer_ros_msgs::StatusCode::OK);
    }
  }
}

bool Node::FinishTrajectory(const int trajectory_id) {
  carto::common::MutexLocker lock(&mutex_);
  return FinishTrajectoryUnderLock(trajectory_id).code ==
         cartographer_ros_msgs::StatusCode::OK;
}

void Node::RunFinalOptimization() {
  {
    carto::common::MutexLocker lock(&mutex_);
    for (const auto& entry : is_active_trajectory_) {
      CHECK(!entry.second);
    }
  }
  // Assuming we are not adding new data anymore, the final optimization
  // can be performed without holding the mutex.
  map_builder_bridge_.RunFinalOptimization();
}

void Node::HandleOdometryMessage(const int trajectory_id,
                                 const std::string& sensor_id,
                                 const nav_msgs::Odometry::ConstPtr& msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).odometry_sampler.Pulse()) {
    return;
  }
  auto sensor_bridge_ptr = map_builder_bridge_.sensor_bridge(trajectory_id);
  auto odometry_data_ptr = sensor_bridge_ptr->ToOdometryData(msg);
  if (odometry_data_ptr != nullptr) {
    extrapolators_.at(trajectory_id).AddOdometryData(*odometry_data_ptr);
  }
  sensor_bridge_ptr->HandleOdometryMessage(sensor_id, msg);
}

void Node::HandleNavSatFixMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::NavSatFix::ConstPtr& msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).fixed_frame_pose_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleNavSatFixMessage(sensor_id, msg);
}

void Node::HandleLandmarkMessage(
    const int trajectory_id, const std::string& sensor_id,
    const cartographer_ros_msgs::LandmarkList::ConstPtr& msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).landmark_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleLandmarkMessage(sensor_id, msg);
}

void Node::HandleImuMessage(const int trajectory_id,
                            const std::string& sensor_id,
                            const sensor_msgs::Imu::ConstPtr& msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).imu_sampler.Pulse()) {
    return;
  }
  auto sensor_bridge_ptr = map_builder_bridge_.sensor_bridge(trajectory_id);
  auto imu_data_ptr = sensor_bridge_ptr->ToImuData(msg);
  if (imu_data_ptr != nullptr) {
    extrapolators_.at(trajectory_id).AddImuData(*imu_data_ptr);
  }
  sensor_bridge_ptr->HandleImuMessage(sensor_id, msg);
}

void Node::HandleLaserScanMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::LaserScan::ConstPtr& msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleLaserScanMessage(sensor_id, msg);
}

void Node::HandleMultiEchoLaserScanMessage(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleMultiEchoLaserScanMessage(sensor_id, msg);
}

void Node::HandlePointCloud2Message(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  carto::common::MutexLocker lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandlePointCloud2Message(sensor_id, msg);
}

void Node::SerializeState(const std::string& filename) {
  carto::common::MutexLocker lock(&mutex_);
  CHECK(map_builder_bridge_.SerializeState(filename))
      << "Could not write state.";
}

void Node::LoadState(const std::string& state_filename,
                     const bool load_frozen_state) {
  carto::common::MutexLocker lock(&mutex_);
  map_builder_bridge_.LoadState(state_filename, load_frozen_state);
}


void Node::SavePoseToFiles()
{
  // time tag in millisecond,for output file
  struct timeval tp;
  gettimeofday(&tp, NULL);
  long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
  
  // file1
  std::ofstream OrigLocalPoseFile;
  std::string path_temp = "/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/Orig_LocPo_TX_" + std::to_string(ms) + ".txt";
  char* orig_local_pose_path = &path_temp[0];    
  double tx, ty, tz, qw, qx, qy, qz, siny, cosy, yaw;
  OrigLocalPoseFile.open(orig_local_pose_path, ios::out);
  for (auto& original_local_pose : original_local_poses_) {
    tx = original_local_pose.x_pose.translation().x();
    ty = original_local_pose.x_pose.translation().y();
    tz = original_local_pose.x_pose.translation().z();
    qw = original_local_pose.x_pose.rotation().w();
    qx = original_local_pose.x_pose.rotation().x();    
    qy = original_local_pose.x_pose.rotation().y();
    qz = original_local_pose.x_pose.rotation().z();
    
    siny = +2.0 * (qw * qz+ qx * qy);
    cosy = +1.0 - 2.0 * (qy * qy + qz * qz);
    yaw = atan2(siny, cosy);    
    
    OrigLocalPoseFile << original_local_pose.pose_tx_index;
    OrigLocalPoseFile << " " << original_local_pose.time;
    OrigLocalPoseFile << " " << original_local_pose.node_id_from_local;
    OrigLocalPoseFile << " " << tx << " " << ty << " " << yaw 
                      << " " << original_local_pose.pose_publisher_tx_index <<std::endl;     
  }
  OrigLocalPoseFile.close();
  LOG(INFO) << "original local pose saved @ " << orig_local_pose_path;
  std::cout << "original local pose saved @ " << orig_local_pose_path << std::endl;
  
  //file2
  std::ofstream OptimGlobPoseFile;
  path_temp = "/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/Opti_GloPo_TX_" + std::to_string(ms) + ".txt";
  char* optim_glob_pose_path = &path_temp[0];    
  OptimGlobPoseFile.open(optim_glob_pose_path, ios::out);
  for (auto& optimized_global_pose : optimized_global_poses_) {    
    tx = optimized_global_pose.x_pose.translation().x();
    ty = optimized_global_pose.x_pose.translation().y();
    tz = optimized_global_pose.x_pose.translation().z();
    qw = optimized_global_pose.x_pose.rotation().w();
    qx = optimized_global_pose.x_pose.rotation().x();    
    qy = optimized_global_pose.x_pose.rotation().y();
    qz = optimized_global_pose.x_pose.rotation().z();
    
    siny = +2.0 * (qw * qz+ qx * qy);
    cosy = +1.0 - 2.0 * (qy * qy + qz * qz);
    yaw = atan2(siny, cosy);    
    
    OptimGlobPoseFile << optimized_global_pose.pose_tx_index;
    OptimGlobPoseFile << " " << optimized_global_pose.time;
    OptimGlobPoseFile << " " << optimized_global_pose.node_id_from_local;
    OptimGlobPoseFile << " " << tx << " " << ty << " " << yaw 
                      << " " << optimized_global_pose.pose_publisher_tx_index <<std::endl;      
  }
  OptimGlobPoseFile.close();
  LOG(INFO) << "optimized global pose saved @ " << optim_glob_pose_path;
  std::cout << "optimized global pose saved @ " << optim_glob_pose_path << std::endl; 
  
  
  //file3
  int final_pose_counter = 0;
  std::ofstream FinalGlobPoseFile;
  path_temp = "/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/Final_GloPo_TX_" + std::to_string(ms) + ".txt";
  char* final_glob_pose_path = &path_temp[0];    
  FinalGlobPoseFile.open(final_glob_pose_path, ios::out);  
  using NodeId = ::cartographer::mapping::NodeId;
  using TrajectoryNodePose = ::cartographer::mapping::TrajectoryNodePose;
  
  ::cartographer::mapping::MapById<NodeId, TrajectoryNodePose> node_poses_for_fuse = map_builder_bridge_.GetNodeListForFusion();
  for (const int trajectory_id : node_poses_for_fuse.trajectory_ids()){
    for (const auto& node_id_data_fuse : node_poses_for_fuse.trajectory(trajectory_id)) {
      final_pose_counter += 1;
      tx = node_id_data_fuse.data.global_pose.translation().x();
      ty = node_id_data_fuse.data.global_pose.translation().y();
      tz = node_id_data_fuse.data.global_pose.translation().z();
      qw = node_id_data_fuse.data.global_pose.rotation().w();
      qx = node_id_data_fuse.data.global_pose.rotation().x();    
      qy = node_id_data_fuse.data.global_pose.rotation().y();
      qz = node_id_data_fuse.data.global_pose.rotation().z();
  
      siny = +2.0 * (qw * qz+ qx * qy);
      cosy = +1.0 - 2.0 * (qy * qy + qz * qz);
      yaw = atan2(siny, cosy);
  
      FinalGlobPoseFile << final_pose_counter;
      FinalGlobPoseFile << " " << node_id_data_fuse.data.constant_pose_data.value().time;
      FinalGlobPoseFile << " " << node_id_data_fuse.id.node_index;
      FinalGlobPoseFile << " " << tx << " " << ty << " " << yaw <<std::endl; 
      // this File lack of pose_publisher_tx_index 
    }
  }
  FinalGlobPoseFile.close();
  LOG(INFO) << "Final global pose saved @ " << final_glob_pose_path;
  std::cout << "Final global pose saved @ " << final_glob_pose_path << std::endl;
  
  //file4
  std::ofstream Loc2MapAdjustPoseFile;
  path_temp = "/home/ubuntu/Documents/Tools/scripts/inputs/0730_glb_optm_rosbag_test/Lo2Map_Adjust_TX_" + std::to_string(ms) + ".txt";
  char* loc2map_adjust_pose_path = &path_temp[0];    
  Loc2MapAdjustPoseFile.open(loc2map_adjust_pose_path, ios::out);
  for (auto& loc2map_adjust_pose : loctomap_adjust_poses_) {    
    tx = loc2map_adjust_pose.x_pose.translation().x();
    ty = loc2map_adjust_pose.x_pose.translation().y();
    tz = loc2map_adjust_pose.x_pose.translation().z();
    qw = loc2map_adjust_pose.x_pose.rotation().w();
    qx = loc2map_adjust_pose.x_pose.rotation().x();    
    qy = loc2map_adjust_pose.x_pose.rotation().y();
    qz = loc2map_adjust_pose.x_pose.rotation().z();
    
    siny = +2.0 * (qw * qz+ qx * qy);
    cosy = +1.0 - 2.0 * (qy * qy + qz * qz);
    yaw = atan2(siny, cosy);    
    
    Loc2MapAdjustPoseFile << loc2map_adjust_pose.pose_tx_index;
    Loc2MapAdjustPoseFile << " " << loc2map_adjust_pose.time;
    Loc2MapAdjustPoseFile << " " << loc2map_adjust_pose.node_id_from_local;
    Loc2MapAdjustPoseFile << " " << tx << " " << ty << " " << yaw 
                          << " " << loc2map_adjust_pose.pose_publisher_tx_index <<std::endl;      
  }
  Loc2MapAdjustPoseFile.close();
  LOG(INFO) << "local to map adjust pose saved @ " << loc2map_adjust_pose_path;
  std::cout << "local to map adjust global pose saved @ " << loc2map_adjust_pose_path << std::endl; 
}

// copied from pbstream_to_ros_map_main.cc
// put it here, cuz save map after Final optimization.
void Node::RunMapSave(const std::string& pbstream_filename, const std::string& map_filestem,
         const double resolution) {
  ::cartographer::io::ProtoStreamReader reader(pbstream_filename);
  ::cartographer::io::ProtoStreamDeserializer deserializer(&reader);

  const auto& pose_graph = deserializer.pose_graph();

  LOG(INFO) << "Loading submap slices from serialized data.";
  std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice>
      submap_slices;
  ::cartographer::mapping::proto::SerializedData proto;
  while (deserializer.ReadNextSerializedData(&proto)) {
    if (proto.has_submap()) {
      const auto& submap = proto.submap();
      const ::cartographer::mapping::SubmapId id{
          submap.submap_id().trajectory_id(),
          submap.submap_id().submap_index()};
      const ::cartographer::transform::Rigid3d global_submap_pose =
          ::cartographer::transform::ToRigid3(
              pose_graph.trajectory(id.trajectory_id)
                  .submap(id.submap_index)
                  .pose());
      FillSubmapSlice(global_submap_pose, submap, &submap_slices[id]);
    }
  }
  CHECK(reader.eof());

  LOG(INFO) << "Generating combined map image from submap slices.";
  auto result =
      ::cartographer::io::PaintSubmapSlices(submap_slices, resolution);

  ::cartographer::io::StreamFileWriter pgm_writer(map_filestem + ".pgm");

  ::cartographer::io::Image image(std::move(result.surface));
  WritePgm(image, resolution, &pgm_writer);

  const Eigen::Vector2d origin(
      -result.origin.x() * resolution,
      (result.origin.y() - image.height()) * resolution);

  ::cartographer::io::StreamFileWriter yaml_writer(map_filestem + ".yaml");
  WriteYaml(resolution, origin, pgm_writer.GetFilename(), &yaml_writer);
  std::cout << "Running combined map saving is finished!" << std::endl;
}








}  //namespace cartographer_ros
