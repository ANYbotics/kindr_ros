#include <iomanip>

#include "multi_dof_joint_trajectory_rviz_plugins/MultiDOFJointTrajectoryDisplay.hpp"


namespace multi_dof_joint_trajectory_rviz_plugins {

MultiDOFJointTrajectoryDisplay::MultiDOFJointTrajectoryDisplay()
: size_transform_rotation_(0.2),
  diameter_arrows_(0.05),
  scale_velocity_linear_(1.0),
  scale_velocity_angular_(1.0),
  scale_acceleration_linear_(1.0),
  scale_acceleration_angular_(1.0),
  alpha_(1.0),
  color_connection_(Ogre::ColourValue(1.0, 1.0, 1.0, alpha_)), // white
  color_velocity_linear_(Ogre::ColourValue(0.4, 0.0, 0.0, alpha_)), // dark red
  color_velocity_angular_(Ogre::ColourValue(0.0, 0.4, 0.0, alpha_)), // dark green
  color_acceleration_linear_(Ogre::ColourValue(1.0, 1.0, 0.0, alpha_)), // yellow
  color_acceleration_angular_(Ogre::ColourValue(0.75, 0.0, 0.75, alpha_)), // purple
  font_size_(0.05),
  show_text_(true)
{
  size_property_transform_rotation_ = new rviz::FloatProperty(
      "Size Transform Rotation", size_transform_rotation_,
      "Size of the axes of the rotation transform.",
      this, SLOT(setSizeTransformRotation()));
  size_property_transform_rotation_->setMin(0);

  diameter_property_arrows_ = new rviz::FloatProperty(
      "Diameter Arrows", diameter_arrows_,
      "Diameter of the arrows.",
      this, SLOT(setDiameterArrows()));
  diameter_property_arrows_->setMin(0);

  scale_property_velocity_linear_ = new rviz::FloatProperty(
      "Scale Velocity Linear", scale_velocity_linear_,
      "Scale of the linear velocity.",
      this, SLOT(setScaleVelocityLinear()));

  scale_property_velocity_angular_ = new rviz::FloatProperty(
      "Scale Velocity Angular", scale_velocity_angular_,
      "Scale of the angular velocity.",
      this, SLOT(setScaleVelocityAngular()));

  scale_property_acceleration_linear_ = new rviz::FloatProperty(
      "Scale Acceleration Linear", scale_acceleration_linear_,
      "Scale of the linear acceleration.",
      this, SLOT(setScaleAccelerationLinear()));

  scale_property_acceleration_angular_ = new rviz::FloatProperty(
      "Scale Acceleration Angular", scale_acceleration_angular_,
      "Scale of the angular acceleration.",
      this, SLOT(setScaleAccelerationAngular()));

  color_property_connection_ = new rviz::ColorProperty(
      "Color Connection", rviz::ogreToQt(color_connection_),
      "Color of connection lines.",
      this, SLOT(setColorConnection()));

  color_property_velocity_linear_ = new rviz::ColorProperty(
      "Color Velocity Linear", rviz::ogreToQt(color_velocity_linear_),
      "Color of the linear velocity.",
      this, SLOT(setColorVelocityLinear()));

  color_property_velocity_angular_ = new rviz::ColorProperty(
      "Color Velocity Angular", rviz::ogreToQt(color_velocity_angular_),
      "Color of the angular velocity.",
      this, SLOT(setColorVelocityAngular()));

  color_property_acceleration_linear_ = new rviz::ColorProperty(
      "Color Acceleration Linear", rviz::ogreToQt(color_acceleration_linear_),
      "Color of the linear acceleration.",
      this, SLOT(setColorAccelerationLinear()));

  color_property_acceleration_angular_ = new rviz::ColorProperty(
      "Color Acceleration Angular", rviz::ogreToQt(color_acceleration_angular_),
      "Color of the angular acceleration.",
      this, SLOT(setColorAccelerationAngular()));

  alpha_property_ = new rviz::FloatProperty(
      "Alpha", alpha_,
      "0 is fully transparent, 1.0 is fully opaque.",
      this, SLOT(setAlpha()));
  alpha_property_->setMin(0);
  alpha_property_->setMax(1);

  font_size_property_ = new rviz::FloatProperty(
      "Font Size", font_size_,
      "Size of the font.",
      this, SLOT(setFontSize()));
  font_size_property_->setMin(0);

  show_text_property_ = new rviz::BoolProperty(
      "Show Caption", show_text_,
      "Enable or disable text rendering.",
      this, SLOT(setShowText()));

  history_length_property_ = new rviz::IntProperty(
      "History Length", 1,
      "Number of prior measurements to display.",
      this, SLOT(setHistoryLength()));
  history_length_property_->setMin(1);
  history_length_property_->setMax(100000);
}

MultiDOFJointTrajectoryDisplay::~MultiDOFJointTrajectoryDisplay()
{

}

void MultiDOFJointTrajectoryDisplay::onInitialize()
{
  MFDClass::onInitialize();
  setHistoryLength();
}

void MultiDOFJointTrajectoryDisplay::reset()
{
  MFDClass::reset();
  points_visuals_.clear();
  connections_visuals_.clear();
}

void MultiDOFJointTrajectoryDisplay::setSizeTransformRotation()
{
  size_transform_rotation_ = size_property_transform_rotation_->getFloat();
  updateSizeTransformRotation();
}

void MultiDOFJointTrajectoryDisplay::setDiameterArrows()
{
  diameter_arrows_ = diameter_property_arrows_->getFloat();
  updateDiameterArrows();
}

void MultiDOFJointTrajectoryDisplay::setScaleVelocityLinear()
{
  scale_velocity_linear_ = scale_property_velocity_linear_->getFloat();
  updateScaleVelocityLinear();
}

void MultiDOFJointTrajectoryDisplay::setScaleVelocityAngular()
{
  scale_velocity_angular_ = scale_property_velocity_angular_->getFloat();
  updateScaleVelocityAngular();
}

void MultiDOFJointTrajectoryDisplay::setScaleAccelerationLinear()
{
  scale_acceleration_linear_ = scale_property_acceleration_linear_->getFloat();
  updateScaleAccelerationLinear();
}

void MultiDOFJointTrajectoryDisplay::setScaleAccelerationAngular()
{
  scale_acceleration_angular_ = scale_property_acceleration_angular_->getFloat();
  updateScaleAccelerationAngular();
}

void MultiDOFJointTrajectoryDisplay::setAlpha()
{
  alpha_ = alpha_property_->getFloat();
  color_connection_.a = alpha_;
  color_velocity_linear_.a = alpha_;
  color_velocity_angular_.a = alpha_;
  color_acceleration_linear_.a = alpha_;
  color_acceleration_angular_.a = alpha_;
  updateColorConnection();
  updateAlphaTransformRotation();
  updateColorVelocityLinear();
  updateColorVelocityAngular();
  updateColorAccelerationLinear();
  updateColorAccelerationAngular();
}

void MultiDOFJointTrajectoryDisplay::setColorConnection()
{
  color_connection_ = rviz::qtToOgre(color_property_connection_->getColor());
  color_connection_.a = alpha_property_->getFloat();
  updateColorConnection();
}

void MultiDOFJointTrajectoryDisplay::setColorVelocityLinear()
{
  color_velocity_linear_ = rviz::qtToOgre(color_property_velocity_linear_->getColor());
  color_velocity_linear_.a = alpha_property_->getFloat();
  updateColorVelocityLinear();
}

void MultiDOFJointTrajectoryDisplay::setColorVelocityAngular()
{
  color_velocity_angular_ = rviz::qtToOgre(color_property_velocity_angular_->getColor());
  color_velocity_angular_.a = alpha_property_->getFloat();
  updateColorVelocityAngular();
}

void MultiDOFJointTrajectoryDisplay::setColorAccelerationLinear()
{
  color_acceleration_linear_ = rviz::qtToOgre(color_property_acceleration_linear_->getColor());
  color_acceleration_linear_.a = alpha_property_->getFloat();
  updateColorAccelerationLinear();
}

void MultiDOFJointTrajectoryDisplay::setColorAccelerationAngular()
{
  color_acceleration_angular_ = rviz::qtToOgre(color_property_acceleration_angular_->getColor());
  color_acceleration_angular_.a = alpha_property_->getFloat();
  updateColorAccelerationAngular();
}

void MultiDOFJointTrajectoryDisplay::setFontSize()
{
  font_size_ = font_size_property_->getFloat();
  updateFontSize();
}

void MultiDOFJointTrajectoryDisplay::setShowText()
{
  show_text_ = show_text_property_->getBool();
  updateShowText();
}


void MultiDOFJointTrajectoryDisplay::setHistoryLength()
{
  points_visuals_.rset_capacity(history_length_property_->getInt());
  connections_visuals_.rset_capacity(history_length_property_->getInt());
}

void MultiDOFJointTrajectoryDisplay::processMessage(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg)
{
  size_transform_rotation_    = size_property_transform_rotation_->getFloat();
  scale_velocity_linear_      = scale_property_velocity_linear_->getFloat();
  scale_velocity_angular_     = scale_property_velocity_angular_->getFloat();
  scale_acceleration_linear_  = scale_property_acceleration_linear_->getFloat();
  scale_acceleration_angular_ = scale_property_acceleration_angular_->getFloat();

  color_connection_           = rviz::qtToOgre(color_property_connection_->getColor());
  color_velocity_linear_      = rviz::qtToOgre(color_property_velocity_linear_->getColor());
  color_velocity_angular_     = rviz::qtToOgre(color_property_velocity_angular_->getColor());
  color_acceleration_linear_  = rviz::qtToOgre(color_property_acceleration_linear_->getColor());
  color_acceleration_angular_ = rviz::qtToOgre(color_property_acceleration_angular_->getColor());
  alpha_ = alpha_property_->getFloat();
  color_connection_.a           = alpha_;
  color_velocity_linear_.a      = alpha_;
  color_velocity_angular_.a     = alpha_;
  color_acceleration_linear_.a  = alpha_;
  color_acceleration_angular_.a = alpha_;

  std::vector<std::vector<std::string>> captions;
  for (unsigned int i = 0; i < msg->points.size(); i++)
  {
    std::vector<std::string> caption_point;
    for (unsigned int j = 0; j < msg->joint_names.size(); j++)
    {
      std::stringstream ss;
      ss << msg->joint_names[j] << ": t" << i << " = " << msg->points[i].time_from_start.toSec() << "s";
      caption_point.push_back(ss.str());
    }
    captions.push_back(caption_point);
  }

  font_size_ = font_size_property_->getFloat();
  show_text_ = show_text_property_->getBool();

  std::vector<boost::shared_ptr<MultiDOFJointTrajectoryPointVisual>> points_visuals;
  std::vector<boost::shared_ptr<MultiDOFJointTrajectoryPointConnectionVisual>> connections_visuals;

  trajectory_msgs::MultiDOFJointTrajectoryPoint last_point;
  trajectory_msgs::MultiDOFJointTrajectoryPoint current_point = msg->points[0];

  // add first point
  points_visuals.push_back(boost::shared_ptr<MultiDOFJointTrajectoryPointVisual>(new MultiDOFJointTrajectoryPointVisual(
      context_->getSceneManager(),
      scene_node_,
      current_point,
      size_transform_rotation_,
      diameter_arrows_,
      scale_velocity_linear_,
      scale_velocity_angular_,
      scale_acceleration_linear_,
      scale_acceleration_angular_,
      alpha_,
      color_velocity_linear_,
      color_velocity_angular_,
      color_acceleration_linear_,
      color_acceleration_angular_,
      captions[0],
      font_size_,
      show_text_)));

  // add second to last points and connections to predecessors
  for (unsigned int i = 1; i < msg->points.size(); i++)
  {
    // go one pose further
    last_point = current_point;
    current_point = msg->points[i];

    // add edge to predecessor
    connections_visuals.push_back(boost::shared_ptr<MultiDOFJointTrajectoryPointConnectionVisual>(new MultiDOFJointTrajectoryPointConnectionVisual(context_->getSceneManager(),
        scene_node_,
        last_point,
        current_point,
        color_connection_)));

    // add pose
    points_visuals.push_back(boost::shared_ptr<MultiDOFJointTrajectoryPointVisual>(new MultiDOFJointTrajectoryPointVisual(
        context_->getSceneManager(),
        scene_node_,
        current_point,
        size_transform_rotation_,
        diameter_arrows_,
        scale_velocity_linear_,
        scale_velocity_angular_,
        scale_acceleration_linear_,
        scale_acceleration_angular_,
        alpha_,
        color_velocity_linear_,
        color_velocity_angular_,
        color_acceleration_linear_,
        color_acceleration_angular_,
        captions[i],
        font_size_,
        show_text_)));
  }

  points_visuals_.push_back(points_visuals);
  connections_visuals_.push_back(connections_visuals);
}

void MultiDOFJointTrajectoryDisplay::updateSizeTransformRotation()
{
  for(size_t i = 0; i < points_visuals_.size(); i++)
  {
    for (unsigned int j = 0; j < points_visuals_[i].size(); j++)
    {
      points_visuals_[i][j]->setSizeTransformRotation(size_transform_rotation_);
    }
  }
}

void MultiDOFJointTrajectoryDisplay::updateDiameterArrows()
{
  for(size_t i = 0; i < points_visuals_.size(); i++)
  {
    for (unsigned int j = 0; j < points_visuals_[i].size(); j++)
    {
      points_visuals_[i][j]->setDiametersArrows(diameter_arrows_);
    }
  }
}

void MultiDOFJointTrajectoryDisplay::updateScaleVelocityLinear()
{
  for(size_t i = 0; i < points_visuals_.size(); i++)
  {
    for (unsigned int j = 0; j < points_visuals_[i].size(); j++)
    {
      points_visuals_[i][j]->setScaleVelocityLinear(scale_velocity_linear_);
    }
  }
}

void MultiDOFJointTrajectoryDisplay::updateScaleVelocityAngular()
{
  for(size_t i = 0; i < points_visuals_.size(); i++)
  {
    for (unsigned int j = 0; j < points_visuals_[i].size(); j++)
    {
      points_visuals_[i][j]->setScaleVelocityAngular(scale_velocity_angular_);
    }
  }
}

void MultiDOFJointTrajectoryDisplay::updateScaleAccelerationLinear()
{
  for(size_t i = 0; i < points_visuals_.size(); i++)
  {
    for (unsigned int j = 0; j < points_visuals_[i].size(); j++)
    {
      points_visuals_[i][j]->setScaleAccelerationLinear(scale_acceleration_linear_);
    }
  }
}

void MultiDOFJointTrajectoryDisplay::updateScaleAccelerationAngular()
{
  for(size_t i = 0; i < points_visuals_.size(); i++)
  {
    for (unsigned int j = 0; j < points_visuals_[i].size(); j++)
    {
      points_visuals_[i][j]->setScaleAccelerationAngular(scale_acceleration_angular_);
    }
  }
}

void MultiDOFJointTrajectoryDisplay::updateColorConnection()
{
  for(size_t i = 0; i < connections_visuals_.size(); i++)
  {
    for (unsigned int j = 0; j < connections_visuals_[i].size(); j++)
    {
      connections_visuals_[i][j]->setColor(color_connection_);
    }
  }
}

void MultiDOFJointTrajectoryDisplay::updateAlphaTransformRotation()
{
  for(size_t i = 0; i < points_visuals_.size(); i++)
  {
    for (unsigned int j = 0; j < points_visuals_[i].size(); j++)
    {
      points_visuals_[i][j]->setAlphaTransformRotation(alpha_);
    }
  }
}

void MultiDOFJointTrajectoryDisplay::updateColorVelocityLinear()
{
  for(size_t i = 0; i < points_visuals_.size(); i++)
  {
    for (unsigned int j = 0; j < points_visuals_[i].size(); j++)
    {
      points_visuals_[i][j]->setColorVelocityLinear(color_velocity_linear_);
    }
  }
}

void MultiDOFJointTrajectoryDisplay::updateColorVelocityAngular()
{
  for(size_t i = 0; i < points_visuals_.size(); i++)
  {
    for (unsigned int j = 0; j < points_visuals_[i].size(); j++)
    {
      points_visuals_[i][j]->setColorVelocityAngular(color_velocity_angular_);
    }
  }
}

void MultiDOFJointTrajectoryDisplay::updateColorAccelerationLinear()
{
  for(size_t i = 0; i < points_visuals_.size(); i++)
  {
    for (unsigned int j = 0; j < points_visuals_[i].size(); j++)
    {
      points_visuals_[i][j]->setColorAccelerationLinear(color_acceleration_linear_);
    }
  }
}

void MultiDOFJointTrajectoryDisplay::updateColorAccelerationAngular()
{
  for(size_t i = 0; i < points_visuals_.size(); i++)
  {
    for (unsigned int j = 0; j < points_visuals_[i].size(); j++)
    {
      points_visuals_[i][j]->setColorAccelerationAngular(color_acceleration_angular_);
    }
  }
}

void MultiDOFJointTrajectoryDisplay::updateFontSize()
{
  for(size_t i = 0; i < points_visuals_.size(); i++)
  {
    for (unsigned int j = 0; j < points_visuals_[i].size(); j++)
    {
      points_visuals_[i][j]->setFontSize(font_size_);
    }
  }
}

void MultiDOFJointTrajectoryDisplay::updateShowText()
{
  for(size_t i = 0; i < points_visuals_.size(); i++)
  {
    for (unsigned int j = 0; j < points_visuals_[i].size(); j++)
    {
      points_visuals_[i][j]->setShowText(show_text_);
    }
  }
}

} // multi_dof_joint_trajectory_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(multi_dof_joint_trajectory_rviz_plugins::MultiDOFJointTrajectoryDisplay, rviz::Display)
