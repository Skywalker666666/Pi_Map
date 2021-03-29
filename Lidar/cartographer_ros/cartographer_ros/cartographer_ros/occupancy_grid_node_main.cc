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

#include <cmath>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cairo/cairo.h"
#include "cartographer/common/mutex.h"
#include "cartographer/common/port.h"
#include "cartographer/io/image.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/id.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/submap.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "gflags/gflags.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"


#include "glog/logging.h"

#include "cartographer_ros_msgs/GetSubmap.h"


DEFINE_double(resolution, 0.05,
              "Resolution of a grid cell in the published occupancy grid.");
DEFINE_double(publish_period_sec, 1.0, "OccupancyGrid publishing period.");

namespace cartographer_ros {
namespace {

using ::cartographer::io::PaintSubmapSlicesResult;
using ::cartographer::io::SubmapSlice;
using ::cartographer::mapping::SubmapId;

class Node {
 public:
  explicit Node(double resolution, double publish_period_sec);
  ~Node() {}

  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

 private:
  void HandleSubmapList(const cartographer_ros_msgs::SubmapList::ConstPtr& msg);
  void DrawAndPublish(const ::ros::WallTimerEvent& timer_event);
  void PublishOccupancyGrid(const std::string& frame_id, const ros::Time& time,
                            const Eigen::Array2f& origin,
                            cairo_surface_t* surface);

  bool submapCallback(cartographer_ros_msgs::GetSubmap::Request &req,
                      cartographer_ros_msgs::GetSubmap::Response &res);


  ::ros::NodeHandle node_handle_;
  const double resolution_;

  ::cartographer::common::Mutex mutex_;
  ::ros::ServiceClient client_ GUARDED_BY(mutex_);
  ::ros::Subscriber submap_list_subscriber_ GUARDED_BY(mutex_);
  ::ros::Publisher occupancy_grid_publisher_ GUARDED_BY(mutex_);
  std::map<SubmapId, SubmapSlice> submap_slices_ GUARDED_BY(mutex_);
  ::ros::WallTimer occupancy_grid_publisher_timer_;
  std::string last_frame_id_;
  ros::Time last_timestamp_;
  ::ros::ServiceServer submap_service_ GUARDED_BY(mutex_);
};

Node::Node(const double resolution, const double publish_period_sec)
    : resolution_(resolution),
      client_(node_handle_.serviceClient<::cartographer_ros_msgs::SubmapQuery>(
          kSubmapQueryServiceName)),
      submap_list_subscriber_(node_handle_.subscribe(
          kSubmapListTopic, kLatestOnlyPublisherQueueSize,
          boost::function<void(
              const cartographer_ros_msgs::SubmapList::ConstPtr&)>(
              [this](const cartographer_ros_msgs::SubmapList::ConstPtr& msg) {
                HandleSubmapList(msg);
              }))),
      occupancy_grid_publisher_(
          node_handle_.advertise<::nav_msgs::OccupancyGrid>(
              kOccupancyGridTopic, kLatestOnlyPublisherQueueSize,
              true /* latched */)),
      occupancy_grid_publisher_timer_(
          node_handle_.createWallTimer(::ros::WallDuration(publish_period_sec),
                                       &Node::DrawAndPublish, this)),
      submap_service_(
          node_handle_.advertiseService("getsubmap", 
                                       &Node::submapCallback, this)) {}

void Node::HandleSubmapList(
    const cartographer_ros_msgs::SubmapList::ConstPtr& msg) {
  ::cartographer::common::MutexLocker locker(&mutex_);

  // We do not do any work if nobody listens.
  if (occupancy_grid_publisher_.getNumSubscribers() == 0) {
    return;
  }

  // Keep track of submap IDs that don't appear in the message anymore.
  std::set<SubmapId> submap_ids_to_delete;
  for (const auto& pair : submap_slices_) {
    submap_ids_to_delete.insert(pair.first);
  }

  for (const auto& submap_msg : msg->submap) {
    const SubmapId id{submap_msg.trajectory_id, submap_msg.submap_index};
    submap_ids_to_delete.erase(id);
    SubmapSlice& submap_slice = submap_slices_[id];
    submap_slice.pose = ToRigid3d(submap_msg.pose);

    LOG(INFO) << "XXXXXXXXXXXXX";  // added by Skywalker

    //submap_slice.pose.Translation();

    submap_slice.metadata_version = submap_msg.submap_version;
    if (submap_slice.surface != nullptr &&
        submap_slice.version == submap_msg.submap_version) {
      continue;
    }

    auto fetched_textures =
        ::cartographer_ros::FetchSubmapTextures(id, &client_);
    if (fetched_textures == nullptr) {
      continue;
    }
    CHECK(!fetched_textures->textures.empty());
    submap_slice.version = fetched_textures->version;

    // We use the first texture only. By convention this is the highest
    // resolution texture and that is the one we want to use to construct the
    // map for ROS.
    const auto fetched_texture = fetched_textures->textures.begin();
    submap_slice.width = fetched_texture->width;
    submap_slice.height = fetched_texture->height;
    submap_slice.slice_pose = fetched_texture->slice_pose;
    submap_slice.resolution = fetched_texture->resolution;
    submap_slice.cairo_data.clear();
    submap_slice.surface = ::cartographer::io::DrawTexture(
        fetched_texture->pixels.intensity, fetched_texture->pixels.alpha,
        fetched_texture->width, fetched_texture->height,
        &submap_slice.cairo_data);
  }

  // Delete all submaps that didn't appear in the message.
  for (const auto& id : submap_ids_to_delete) {
    submap_slices_.erase(id);
  }

  last_timestamp_ = msg->header.stamp;
  last_frame_id_ = msg->header.frame_id;
}

void Node::DrawAndPublish(const ::ros::WallTimerEvent& unused_timer_event) {
  if (submap_slices_.empty() || last_frame_id_.empty()) {
    return;
  }

  ::cartographer::common::MutexLocker locker(&mutex_);
  auto painted_slices = PaintSubmapSlices(submap_slices_, resolution_);
  std::unique_ptr<nav_msgs::OccupancyGrid> msg_ptr = CreateOccupancyGridMsg(
      painted_slices, resolution_, last_frame_id_, last_timestamp_);
  occupancy_grid_publisher_.publish(*msg_ptr);
}


bool Node::submapCallback(cartographer_ros_msgs::GetSubmap::Request &req,
                          cartographer_ros_msgs::GetSubmap::Response &res){
  int submap_index = req.submap_index; 
  LOG(INFO) << "@@@@@@@@@ It called submap : " << submap_index;  // added by Skywalker
  //auto painted_slices = PaintSubmapSlices(&submap_slices_, resolution_, submap_index);
  auto painted_slices = PaintSubmapSlices2(submap_slices_, resolution_, submap_index);
  //LOG(INFO) << "@@@@@@@@@ It called submap : " << submap_slices_;  // added by Skywalker
  //for(auto item = submap_slices_.begin(); item!=submap_slices_.end(); item++)
  //{
  //  LOG(INFO) << "!!!!!!!!!!!!!!!!!KEY VALUE: " << item->first;
  //}
  //KEY VALUE: (0, 0)

  nav_msgs::OccupancyGrid occupancy_grid;
  cairo_surface_t *surface = painted_slices.surface.get();
  Eigen::Array2f& origin=painted_slices.origin;
  const int width = cairo_image_surface_get_width(surface);
  const int height = cairo_image_surface_get_height(surface);
  occupancy_grid.header.stamp = ros::Time::now();
  occupancy_grid.header.frame_id = "map";    //all submaps need to be presented under unified coordinate
  occupancy_grid.info.map_load_time = ros::Time::now();
  occupancy_grid.info.resolution = resolution_;

  occupancy_grid.info.width = width;
  occupancy_grid.info.height = height;
  // TO DO: position and orientation of origin 

  //……………………………………………………………………………………………………………………
  const uint32_t* pixel_data = reinterpret_cast<uint32_t*>(cairo_image_surface_get_data(surface));
  occupancy_grid.data.reserve(width * height);
  for (int y = height - 1; y >= 0; --y) {
    for (int x = 0; x < width; ++x) {
      const uint32_t packed = pixel_data[y * width + x];
      const unsigned char color = packed >> 16;
      const unsigned char observed = packed >> 8;
      const int value = observed == 0 ? -1
              : ::cartographer::common::RoundToInt((1. - color / 255.) * 100.);
      CHECK_LE(-1, value); CHECK_GE(100, value);
      occupancy_grid.data.push_back(value);
    }
  }
  res.map=occupancy_grid;
  /*
  map: 
  header: 
    seq: 0
    stamp: 
      secs: 1551150319
      nsecs: 889773769
    frame_id: "map"
  info: 
    map_load_time: 
      secs: 1551150319
      nsecs: 889773769
    resolution: 0.0500000007451
    width: 0
    height: 0
    origin: 
      position: 
        x: 0.0
        y: 0.0
        z: 0.0
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 0.0
  */


  // REFERENCE: map_server/src/map_saver.cpp
  // we can use req.xxxx for true and false
  if(req.write_map) {   // Default option: continuously save the map 
      std::string mapname_ ="map" ;
      mapname_.append(boost::lexical_cast<std::string>(submap_index));
      ROS_INFO("Received a %d X %d map @ %.3f m/pix", width, height, resolution_);
      std::string mapdatafile = mapname_ + ".pgm";
      ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
      FILE* out = fopen(mapdatafile.c_str(), "w");
      if (!out){
        ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
        return false; 
      }
      fprintf(out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n", resolution_, width, height);
      // same way used in ROS map_server maps saving
      //……………………………………………………………………………………………………………………
      for(unsigned int y = 0; y < height; y++) {
        for(unsigned int x =0; x < width; x++) {
          unsigned int i = x + (height - y - 1) * width;
          if ( occupancy_grid.data[i] >= 0 && occupancy_grid.data[i] <= req.th_free_) {//free normally 0.45
            fputc(254, out);
          } else if (occupancy_grid.data[i] <= 100 && occupancy_grid.data[i] >= req.th_occupied_) {//occ normally 0.55
            fputc(000, out);
          } else {
            fputc(205, out);
          }//if
        }//for x
      }//for y


  }
  return true;
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  ::ros::init(argc, argv, "cartographer_occupancy_grid_node");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  ::cartographer_ros::Node node(FLAGS_resolution, FLAGS_publish_period_sec);

  ::ros::spin();
  ::ros::shutdown();
}
