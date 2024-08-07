/*
 * Copyright (c) 2014, Peter Fankhauser, Christian Gehring, Hannes Sommer, Paul Furgale, Remo Diethelm
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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Peter Fankhauser, Christian Gehring, Hannes Sommer, Paul Furgale,
 * Remo Diethelm BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <iostream>

#include <Eigen/Core>

#include <gtest/gtest.h>

#include "kindr/common/gtest_eigen.hpp"
#include "kindr/poses/Pose.hpp"
#include "kindr_ros/RosGeometryMsgRotation.hpp"

// ROS
#ifndef ROS2_BUILD
#include <geometry_msgs/Quaternion.h>
#else /* ROS2_BUILD */
#include <geometry_msgs/msg/quaternion.hpp>
#endif /* ROS2_BUILD */

TEST(RosGeometryMsgRotationQuaternionEigen, convertFromRosGeometryMsg) {
  const kindr::RotationQuaternionPD referenceQuaternion(0.113, 0.071, -0.924, 0.35835);

#ifndef ROS2_BUILD
  geometry_msgs::Quaternion geometryQuaternionMsg;
#else  /* ROS2_BUILD */
  geometry_msgs::msg::Quaternion geometryQuaternionMsg;
#endif /* ROS2_BUILD */
  geometryQuaternionMsg.x = referenceQuaternion.x();
  geometryQuaternionMsg.y = referenceQuaternion.y();
  geometryQuaternionMsg.z = referenceQuaternion.z();
  geometryQuaternionMsg.w = referenceQuaternion.w();

  kindr::RotationQuaternionPD rotationQuaternion;
  kindr_ros::convertFromRosGeometryMsg(geometryQuaternionMsg, rotationQuaternion);

  EXPECT_TRUE(rotationQuaternion.isNear(referenceQuaternion, 1e-8));
}

TEST(RosGeometryMsgRotationQuaternionEigen, convertToRosGeometryMsg) {
  const kindr::RotationQuaternionPD referenceQuaternion(0.212, 0.0421, -0.958, 0.1885);

  kindr::RotationQuaternionPD rotationQuaternion(referenceQuaternion);

#ifndef ROS2_BUILD
  geometry_msgs::Quaternion geometryQuaternionMsg;
#else  /* ROS2_BUILD */
  geometry_msgs::msg::Quaternion geometryQuaternionMsg;
#endif /* ROS2_BUILD */
  kindr_ros::convertToRosGeometryMsg(rotationQuaternion, geometryQuaternionMsg);

  EXPECT_NEAR(geometryQuaternionMsg.x, referenceQuaternion.x(), 1e-8);
  EXPECT_NEAR(geometryQuaternionMsg.y, referenceQuaternion.y(), 1e-8);
  EXPECT_NEAR(geometryQuaternionMsg.z, referenceQuaternion.z(), 1e-8);
  EXPECT_NEAR(geometryQuaternionMsg.w, referenceQuaternion.w(), 1e-8);
}
