/*
 * Copyright (c) 2016, Michael Pantic [mpantic@student.ethz.ch],
 *                     Autonomous Systems Lab, ETH Zurich, Switzerland
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/ros.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include "serial/serial.h" //need to install ros-indigo-serial
#include <Eigen/Core>
#include <eigen_conversions/eigen_msg.h>

Eigen::Quaterniond g_orientation;
bool g_got_orientation;

void odometry_callback(const nav_msgs::OdometryConstPtr &msg) {
  tf::quaternionMsgToEigen(msg->pose.pose.orientation, g_orientation);
  g_got_orientation = true;
}

void do_tilt_compensation(double z_body, Eigen::Vector3d *p_depth_tilted) {
  Eigen::Quaterniond p(0.0, 0.0, 0.0, z_body);
  Eigen::Quaterniond rotated_p = g_orientation * p * g_orientation.inverse();
  *p_depth_tilted = rotated_p.vec();
}

int main(int argc, char **argv) {

  std::string name = "garmin_lidar_node";
  ros::init(argc, argv, name);
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  geometry_msgs::Vector3Stamped msg;
  visualization_msgs::Marker marker;
  Eigen::Vector3d p_depth_tilted;
  bool do_tilting_compensation;
  bool plot_tilted_vector;

  // TODO switch from full odometry to orientation only message
  ros::Subscriber odometry_sub = nh.subscribe("odometry", 1,
                                              &odometry_callback);

  ros::Publisher depth_pub = nh.advertise<geometry_msgs::Vector3Stamped>("depth",
                                                                         1000);
  ros::Publisher depth_compensated_pub = nh.advertise<geometry_msgs::Vector3Stamped>(
      "depth_compensated", 1000);
  ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>(
      "depth_visualization_marker", 0);

  nh_private.param("do_tilting_compensation", do_tilting_compensation, true);
  nh_private.param("plot_tilted_vector", plot_tilted_vector, true);

  serial::Serial my_serial("/dev/ttyUSB1", 1000000, serial::Timeout::simpleTimeout(1000));
  if (my_serial.isOpen()) {
    ROS_WARN("PORT OPENED");
    my_serial.write("S");
  }

  ros::Time start, end;
  ros::Duration roundtrip;

  // init marker message
  if (plot_tilted_vector) {
    marker.header.frame_id = "garmin_lidar";
    marker.ns = "";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    // arrow tail in the origin of 'frame_id'
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    // pitch 90 deg: arrow length is expressed in marker.scale.x
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.70710;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.70710;
    // for now use zero length
    marker.scale.x = 0.0;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    // yellow arrow
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.81;
    marker.color.b = 0.129;
  }

  while (ros::ok() && my_serial.isOpen()) {
    start = ros::Time::now();
    std::string line = my_serial.readline();

    //valid line
    if (line[0] == 'T' && line[5] == 'M' && line.length() == 9) {
      uint8_t buffer[9];
      std::copy(line.begin(), line.end(), buffer);

      //reconstruct timing
      uint32_t time =
          (uint32_t) buffer[1] | (uint32_t) buffer[2] << 8 | (uint32_t) buffer[3] << 16 | (uint32_t) buffer[4] << 24;

      //reconstruct measurement
      uint16_t distance = (uint16_t) buffer[6] | (uint16_t) buffer[7] << 8;
      end = ros::Time::now();
      roundtrip = end - start;
      roundtrip.fromNSec(roundtrip.toNSec() / 2);

      msg.header.stamp = start + roundtrip;
      msg.vector.x = 0;
      msg.vector.y = 0;
      msg.vector.z = distance / 100.0;
      depth_pub.publish(msg);

      // marker message
      if (plot_tilted_vector) {
        marker.header.stamp = ros::Time();
        // use raw data, TF will take care of rotation to visualize it in Rviz
        marker.scale.x = msg.vector.z;
        vis_pub.publish(marker);
      }

      // compensate measurement using attitude
      if (do_tilting_compensation) {
        do_tilt_compensation(msg.vector.z, &p_depth_tilted);
        tf::vectorEigenToMsg(p_depth_tilted, msg.vector);
        depth_compensated_pub.publish(msg);
      }

      ros::spinOnce();
    }
  }

  return 0;
}
