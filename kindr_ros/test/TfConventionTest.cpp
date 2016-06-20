/*
 * tf_kindr_test.cpp
 *
 *  Created on: Jun 17, 2016
 *      Author: Christian Gehring
 */

#include <gtest/gtest.h>
#include <kindr/Core>
#include <kindr/rotations/gtest_rotations.hpp>
#include <tf/tf.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace kindr {

template<>
class ConversionTraits<tf::Quaternion, tf::Vector3, double> {
  typedef tf::Quaternion Rotation;
  typedef tf::Vector3 Vector;
 public:

  inline static void convertKindrRotationQuaternionToRotation(Rotation& out, const kindr::RotationQuaternion<double>& in) {
    kindr::RotationQuaternion<double> in2 = in;
    out = tf::Quaternion(in2.x(), in2.y(), in2.z(), in2.w());
  }

  inline static void convertVelocityVector(Vector& out, Rotation& rot, const Eigen::Matrix<double,3,1>& in) {
    out = tf::Vector3(in.x(), in.y(), in.z());
  }

  inline static void getRotationMatrixFromRotation(Eigen::Matrix3d& rotationMatrix, const Rotation& quaternion) {
    tf::Matrix3x3 tfRotationmatrix;
    tfRotationmatrix.setRotation(quaternion);
    for (int i=0; i<3; i++) {
      rotationMatrix(i, 0) = tfRotationmatrix.getRow(i).x();
      rotationMatrix(i, 1) = tfRotationmatrix.getRow(i).y();
      rotationMatrix(i, 2) = tfRotationmatrix.getRow(i).z();
    }
  }

  inline static void rotateVector(Eigen::Matrix<double,3,1>& A_r, const Rotation& rotationBToA, const Eigen::Matrix<double,3,1>& B_r) {
    tf::Vector3 A_v = tf::quatRotate(rotationBToA, tf::Vector3(B_r.x(), B_r.y(), B_r.z()));
    A_r.x() = A_v.x();
    A_r.y() = A_v.y();
    A_r.z() = A_v.z();
  }

//  inline static void boxPlus(Rotation& res, const Rotation& rot, const tf::Vector3& velocity) {
//    // todo
//    res = tf::Quaternion(0, 1,0, 0);
//  }

  inline static void testRotation(const Rotation& expected, const Rotation& actual) {
    EXPECT_NEAR(expected.w(), actual.w(), 1.0e-6);
    EXPECT_NEAR(expected.x(), actual.x(), 1.0e-6);
    EXPECT_NEAR(expected.y(), actual.y(), 1.0e-6);
    EXPECT_NEAR(expected.z(), actual.z(), 1.0e-6);
  }

};


template<>
class ConversionTraits<tf::Matrix3x3, tf::Vector3, double> {
  typedef tf::Matrix3x3 Rotation;
  typedef tf::Vector3 Vector;
 public:

  inline static void convertKindrRotationQuaternionToRotation(Rotation& out, const kindr::RotationQuaternion<double>& in) {
    kindr::RotationQuaternion<double> in2 = in;
    out.setRotation(tf::Quaternion(in2.x(), in2.y(), in2.z(), in2.w()));
  }

  inline static void convertVelocityVector(Vector& out, Rotation& rot, const Eigen::Matrix<double,3,1>& in) {
    out = tf::Vector3(in.x(), in.y(), in.z());
  }

  inline static void getRotationMatrixFromRotation(Eigen::Matrix3d& rotationMatrix, const Rotation& matrix) {
    for (int i=0; i<3; i++) {
      rotationMatrix(i, 0) = matrix.getRow(i).x();
      rotationMatrix(i, 1) = matrix.getRow(i).y();
      rotationMatrix(i, 2) = matrix.getRow(i).z();
    }
  }

  inline static void rotateVector(Eigen::Matrix<double,3,1>& A_r, const Rotation& rotationBToA, const Eigen::Matrix<double,3,1>& B_r) {
    tf::Vector3 B_v(B_r.x(), B_r.y(), B_r.z());
    tf::Vector3 A_v = rotationBToA*B_v;
    A_r.x() = A_v.x();
    A_r.y() = A_v.y();
    A_r.z() = A_v.z();
  }

//  inline static void boxPlus(Rotation& res, const Rotation& rot, const tf::Vector3& velocity) {
//    // todo
//    res = tf::Quaternion(0, 1,0, 0);
//  }

  inline static void testRotation(const Rotation& expected, const Rotation& actual) {
    for (int i=0; i<3; i++) {
      EXPECT_NEAR(expected.getRow(i).x(), actual.getRow(i).x(), 1.0e-6);
      EXPECT_NEAR(expected.getRow(i).y(), actual.getRow(i).y(), 1.0e-6);
      EXPECT_NEAR(expected.getRow(i).z(), actual.getRow(i).z(), 1.0e-6);
    }
  }
};


} // namespace kindr
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TEST(TfConventionTest, Concatenation) {
  kindr::ConventionTest<tf::Quaternion, tf::Vector3, double>::testConcatenation();
  kindr::ConventionTest<tf::Matrix3x3, tf::Vector3, double>::testConcatenation();
}

TEST(TfConventionTest, Rotation) {
  kindr::ConventionTest<tf::Quaternion, tf::Vector3, double>::testRotationMatrix();
  kindr::ConventionTest<tf::Matrix3x3, tf::Vector3, double>::testRotationMatrix();
}

//TEST(TfConventionTest, BoxPlus) {
//  kindr::ConventionTest<tf::Quaternion, tf::Vector3, double>::testBoxPlus();
//}

TEST(TfConventionTest, GeometricalInterpretation) {
  kindr::ConventionTest<tf::Quaternion, tf::Vector3, double>::testGeometricalInterpretation();
  kindr::ConventionTest<tf::Matrix3x3, tf::Vector3, double>::testGeometricalInterpretation();
}
