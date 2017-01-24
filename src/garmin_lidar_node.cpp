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
#include "serial/serial.h" //need to install ros-indigo-serial

int main(int argc, char **argv) {

  std::string name = "garmin_lidar_node";
  ros::init(argc, argv, name);
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  ros::Publisher depth_pub = nh.advertise<geometry_msgs::Vector3Stamped>("depth", 1000);


  serial::Serial my_serial("/dev/ttyUSB1", 1000000, serial::Timeout::simpleTimeout(1000));
  if (my_serial.isOpen()) {
    ROS_WARN("PORT OPENED");
    my_serial.write("S");
  }

  ros::Time start, end;
  ros::Duration roundtrip;

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
      roundtrip.fromNSec(roundtrip.toNSec()/2);

      geometry_msgs::Vector3Stamped msg;
      msg.header.stamp = start + roundtrip;
      msg.vector.x = 0;
      msg.vector.y = 0;
      msg.vector.z = distance/100.0;
      depth_pub.publish(msg);

      ros::spinOnce();
    }
  }

  return 0;
}
