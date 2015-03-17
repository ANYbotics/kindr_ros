#pragma once

#ifndef Q_MOC_RUN
#include <vector>

#include <boost/circular_buffer.hpp>

#include <rviz/message_filter_display.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#endif

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/parse_color.h>
#include <rviz/frame_manager.h>

#include "multi_dof_joint_trajectory_rviz_plugins/MultiDOFJointTrajectoryPointConnectionVisual.hpp"
#include "multi_dof_joint_trajectory_rviz_plugins/MultiDOFJointTrajectoryPointVisual.hpp"


namespace multi_dof_joint_trajectory_rviz_plugins {

class MultiDOFJointTrajectoryDisplay: public rviz::MessageFilterDisplay<trajectory_msgs::MultiDOFJointTrajectory>
{
Q_OBJECT
public:
  MultiDOFJointTrajectoryDisplay();
  virtual ~MultiDOFJointTrajectoryDisplay();


protected:
  virtual void onInitialize();
  virtual void reset();


private Q_SLOTS:
  void setSizeTransformRotation();
  void setDiameterArrows();
  void setScaleVelocityLinear();
  void setScaleVelocityAngular();
  void setScaleAccelerationLinear();
  void setScaleAccelerationAngular();

  void setAlpha();
  void setColorConnection();
  void setColorVelocityLinear();
  void setColorVelocityAngular();
  void setColorAccelerationLinear();
  void setColorAccelerationAngular();

  void setFontSize();
  void setShowText();

  void setHistoryLength();


private:
  void processMessage(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg);

  void updateSizeTransformRotation();
  void updateDiameterArrows();
  void updateScaleVelocityLinear();
  void updateScaleVelocityAngular();
  void updateScaleAccelerationLinear();
  void updateScaleAccelerationAngular();

  void updateColorConnection();
  void updateAlphaTransformRotation();
  void updateColorVelocityLinear();
  void updateColorVelocityAngular();
  void updateColorAccelerationLinear();
  void updateColorAccelerationAngular();

  void updateFontSize();
  void updateShowText();

  boost::circular_buffer<std::vector<boost::shared_ptr<MultiDOFJointTrajectoryPointVisual>>> points_visuals_;
  boost::circular_buffer<std::vector<boost::shared_ptr<MultiDOFJointTrajectoryPointConnectionVisual>>> connections_visuals_;

  rviz::FloatProperty* size_property_transform_rotation_;
  rviz::FloatProperty* diameter_property_arrows_;
  rviz::FloatProperty* scale_property_velocity_linear_;
  rviz::FloatProperty* scale_property_velocity_angular_;
  rviz::FloatProperty* scale_property_acceleration_linear_;
  rviz::FloatProperty* scale_property_acceleration_angular_;

  rviz::ColorProperty* color_property_connection_;
  rviz::ColorProperty* color_property_velocity_linear_;
  rviz::ColorProperty* color_property_velocity_angular_;
  rviz::ColorProperty* color_property_acceleration_linear_;
  rviz::ColorProperty* color_property_acceleration_angular_;

  rviz::FloatProperty* alpha_property_;

  rviz::FloatProperty* font_size_property_;
  rviz::BoolProperty* show_text_property_;

  rviz::IntProperty* history_length_property_;

  float size_transform_rotation_;
  float diameter_arrows_;
  float scale_velocity_linear_;
  float scale_velocity_angular_;
  float scale_acceleration_linear_;
  float scale_acceleration_angular_;

  float alpha_;
  Ogre::ColourValue color_connection_;
  Ogre::ColourValue color_velocity_linear_;
  Ogre::ColourValue color_velocity_angular_;
  Ogre::ColourValue color_acceleration_linear_;
  Ogre::ColourValue color_acceleration_angular_;

  float font_size_;
  bool show_text_;
};

} // multi_dof_joint_trajectory_rviz_plugins
