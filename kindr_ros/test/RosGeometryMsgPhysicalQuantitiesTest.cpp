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
#include "kindr/phys_quant/PhysicalQuantities.hpp"
#include "kindr_ros/RosGeometryMsgPhysicalQuantities.hpp"

// ROS
#ifndef ROS2_BUILD
#include <geometry_msgs/Pose.h>
#else /* ROS2_BUILD */
#include <geometry_msgs/msg/pose.hpp>
#endif /* ROS2_BUILD */

TEST(RosGeometryMsgVectorEigen, convertFromRosGeometryMsg) {
  const kindr::VectorTypeless3D referenceVector(-9.12, -25.5, 0.6);

#ifndef ROS2_BUILD
  geometry_msgs::Vector3 geometryVector3Msg;
#else  /* ROS2_BUILD */
  geometry_msgs::msg::Vector3 geometryVector3Msg;
#endif /* ROS2_BUILD */
  geometryVector3Msg.x = referenceVector.x();
  geometryVector3Msg.y = referenceVector.y();
  geometryVector3Msg.z = referenceVector.z();

  kindr::VectorTypeless3D vector;
  kindr_ros::convertFromRosGeometryMsg(geometryVector3Msg, vector);

  kindr::expectNear(vector.toImplementation(), referenceVector.toImplementation(), 1e-8, KINDR_SOURCE_FILE_POS);
}

TEST(RosGeometryMsgVectorEigen, convertToRosGeometryMsg) {
  const kindr::VectorTypeless3D referenceVector(0.3, -1.1, -0.6);

  kindr::VectorTypeless3D vector(referenceVector);

#ifndef ROS2_BUILD
  geometry_msgs::Vector3 geometryVectorMsg;
#else  /* ROS2_BUILD */
  geometry_msgs::msg::Vector3 geometryVectorMsg;
#endif /* ROS2_BUILD */
  kindr_ros::convertToRosGeometryMsg(vector, geometryVectorMsg);

  EXPECT_NEAR(geometryVectorMsg.x, referenceVector.x(), 1e-8);
  EXPECT_NEAR(geometryVectorMsg.y, referenceVector.y(), 1e-8);
  EXPECT_NEAR(geometryVectorMsg.z, referenceVector.z(), 1e-8);
}

TEST(RosGeometryMsgPositionEigen, convertFromRosGeometryPoint32Msg) {
  const kindr::Position3F referenceTranslation(-9.3, 2.5, 5.6);

#ifndef ROS2_BUILD
  geometry_msgs::Point32 geometryPointMsg;
#else  /* ROS2_BUILD */
  geometry_msgs::msg::Point32 geometryPointMsg;
#endif /* ROS2_BUILD */
  geometryPointMsg.x = referenceTranslation.x();
  geometryPointMsg.y = referenceTranslation.y();
  geometryPointMsg.z = referenceTranslation.z();

  kindr::Position3F position;
  kindr_ros::convertFromRosGeometryMsg(geometryPointMsg, position);

  kindr::expectNear(position.toImplementation(), referenceTranslation.toImplementation(), 1e-8, KINDR_SOURCE_FILE_POS);
}

TEST(RosGeometryMsgPositionEigen, convertToRosGeometryPoint32Msg) {
  const kindr::Position3F referenceTranslation(0.3, -1.1, -0.6);

  kindr::Position3F position(referenceTranslation);

#ifndef ROS2_BUILD
  geometry_msgs::Point32 geometryPointMsg;
#else  /* ROS2_BUILD */
  geometry_msgs::msg::Point32 geometryPointMsg;
#endif /* ROS2_BUILD */
  kindr_ros::convertToRosGeometryMsg(position, geometryPointMsg);

  EXPECT_NEAR(geometryPointMsg.x, referenceTranslation.x(), 1e-8);
  EXPECT_NEAR(geometryPointMsg.y, referenceTranslation.y(), 1e-8);
  EXPECT_NEAR(geometryPointMsg.z, referenceTranslation.z(), 1e-8);
}

TEST(RosGeometryMsgPositionEigen, convertFromRosGeometryPointMsg) {
  const kindr::Position3D referenceTranslation(19.3, -2.5, 5.6);

#ifndef ROS2_BUILD
  geometry_msgs::Point geometryPointMsg;
#else  /* ROS2_BUILD */
  geometry_msgs::msg::Point geometryPointMsg;
#endif /* ROS2_BUILD */
  geometryPointMsg.x = referenceTranslation.x();
  geometryPointMsg.y = referenceTranslation.y();
  geometryPointMsg.z = referenceTranslation.z();

  kindr::Position3D position;
  kindr_ros::convertFromRosGeometryMsg(geometryPointMsg, position);

  kindr::expectNear(position.toImplementation(), referenceTranslation.toImplementation(), 1e-8, KINDR_SOURCE_FILE_POS);
}

TEST(RosGeometryMsgPositionEigen, convertToRosGeometryPointMsg) {
  const kindr::Position3D referenceTranslation(0.3, -1.1, -0.6);

  kindr::Position3D position(referenceTranslation);

#ifndef ROS2_BUILD
  geometry_msgs::Point geometryPointMsg;
#else  /* ROS2_BUILD */
  geometry_msgs::msg::Point geometryPointMsg;
#endif /* ROS2_BUILD */
  kindr_ros::convertToRosGeometryMsg(position, geometryPointMsg);

  EXPECT_NEAR(geometryPointMsg.x, referenceTranslation.x(), 1e-8);
  EXPECT_NEAR(geometryPointMsg.y, referenceTranslation.y(), 1e-8);
  EXPECT_NEAR(geometryPointMsg.z, referenceTranslation.z(), 1e-8);
}
