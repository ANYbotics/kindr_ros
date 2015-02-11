/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

#include "kindr_rviz_plugins/VectorAtPositionDisplay.hpp"
#include "kindr_rviz_plugins/VectorAtPositionVisual.hpp"

namespace kindr_rviz_plugins {

// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
VectorAtPositionDisplay::VectorAtPositionDisplay()
: lengthScale_(1.0f),
  widthScale_(1.0f),
  showText_(true),
  color_(Ogre::ColourValue::Black),
  alpha_(1.0)
{
  length_scale_property_ = new rviz::FloatProperty("Length scale", 1.0,
                                                   "Scale of the length of the vector.",
                                                   this, SLOT(updateScale()));

  width_scale_property_ = new rviz::FloatProperty("Width scale", 1.0,
                                                   "Scale of the width of the vector.",
                                                   this, SLOT(updateScale()));
  width_scale_property_->setMin(0);

  show_text_property_ = new rviz::BoolProperty("Show text", true,
                                             "Enable or disable text rendering.",
                                             this, SLOT(updateShowText()));

  color_property_ = new rviz::ColorProperty("Color", QColor(0, 0, 0),
                                             "Color to draw the vector (if not defined by vector type).",
                                             this, SLOT(updateColorAndAlpha()));

  alpha_property_ = new rviz::FloatProperty("Alpha", 1.0,
                                             "0 is fully transparent, 1.0 is fully opaque.",
                                             this, SLOT(updateColorAndAlpha()));

  history_length_property_ = new rviz::IntProperty("History Length", 1,
                                                    "Number of prior measurements to display.",
                                                    this, SLOT(updateHistoryLength()));
  history_length_property_->setMin(1);
  history_length_property_->setMax(100000);
}

// After the top-level rviz::Display::initialize() does its own setup,
// it calls the subclass's onInitialize() function. This is where we
// instantiate all the workings of the class. We make sure to also
// call our immediate super-class's onInitialize() function, since it
// does important stuff setting up the message filter.
//
//  Note that "MFDClass" is a typedef of
// ``MessageFilterDisplay<message type>``, to save typing that long
// templated class name every time you need to refer to the
// superclass.
void VectorAtPositionDisplay::onInitialize()
{
  MFDClass::onInitialize();
  updateHistoryLength();
}

VectorAtPositionDisplay::~VectorAtPositionDisplay()
{
}

// Clear the visuals by deleting their objects.
void VectorAtPositionDisplay::reset()
{
  MFDClass::reset();
  visuals_.clear();
}

// Set the scale.
void VectorAtPositionDisplay::updateScale()
{
  lengthScale_ = length_scale_property_->getFloat();
  widthScale_ = width_scale_property_->getFloat();

  for(size_t i = 0; i < visuals_.size(); i++)
  {
    visuals_[i]->setScalingFactors(lengthScale_, widthScale_);
  }
}

// Show text.
void VectorAtPositionDisplay::updateShowText()
{
  showText_ = show_text_property_->getBool();

  for(size_t i = 0; i < visuals_.size(); i++)
  {
    visuals_[i]->setShowText(showText_);
  }
}

// Set the current color and alpha values for each visual.
void VectorAtPositionDisplay::updateColorAndAlpha()
{
  color_ = color_property_->getOgreColor();
  alpha_ = alpha_property_->getFloat();

  for(size_t i = 0; i < visuals_.size(); i++)
  {
    visuals_[i]->setColor(color_.r, color_.g, color_.b, alpha_);
  }
}

// Set the number of past visuals to show.
void VectorAtPositionDisplay::updateHistoryLength()
{
  visuals_.rset_capacity(history_length_property_->getInt());
}

// This is our callback to handle an incoming message.
void VectorAtPositionDisplay::processMessage(const kindr_msgs::VectorAtPosition::ConstPtr& msg)
{
  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this VectorAtPosition message.
  Ogre::Vector3 arrowPosition;
  Ogre::Quaternion arrowOrientation;

  // If the position has a different frame than the vector
  if (msg->position_frame_id.empty() || msg->position_frame_id == msg->header.frame_id)
  {
    // Get arrow position and orientation
    if(!context_->getFrameManager()->getTransform(msg->header.frame_id, msg->header.stamp, arrowPosition, arrowOrientation))
    {
      ROS_ERROR("Error transforming from frame '%s' to frame '%s'", msg->position_frame_id.c_str(), qPrintable(fixed_frame_));
      return;
    }
  }
  else
  {
    // Get arrow position
    Ogre::Quaternion dummyOrientation;
    if(!context_->getFrameManager()->getTransform(msg->position_frame_id, msg->header.stamp, arrowPosition, dummyOrientation))
    {
      ROS_ERROR("Error transforming from frame '%s' to frame '%s'", msg->position_frame_id.c_str(), qPrintable(fixed_frame_));
      return;
    }

    // Get arrow orientation
    Ogre::Vector3 dummyPosition;
    if(!context_->getFrameManager()->getTransform(msg->header.frame_id, msg->header.stamp, dummyPosition, arrowOrientation))
    {
      ROS_ERROR("Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
      return;
    }
  }
  arrowPosition += Ogre::Vector3(msg->position.x, msg->position.y, msg->position.z);

  // We are keeping a circular buffer of visual pointers. This gets
  // the next one, or creates and stores it if the buffer is not full
  boost::shared_ptr<VectorAtPositionVisual> visual;
  if(visuals_.full())
  {
    visual = visuals_.front();
  }
  else
  {
    visual.reset(new VectorAtPositionVisual(context_->getSceneManager(), scene_node_));
  }

  // Now set or update the contents of the chosen visual.
  visual->setMessage(msg);
  visual->setArrowPosition(arrowPosition); // position is taken from position in msg
  visual->setArrowOrientation(arrowOrientation); // orientation is taken from vector in msg

  lengthScale_ = length_scale_property_->getFloat();
  widthScale_ = width_scale_property_->getFloat();
  visual->setScalingFactors(lengthScale_, widthScale_);
  showText_ = show_text_property_->getBool();
  visual->setShowText(showText_);
  alpha_ = alpha_property_->getFloat();
  visual->setColor(color_.r, color_.g, color_.b, alpha_);

  // And send it to the end of the circular buffer
  visuals_.push_back(visual);
}

} // kindr_rviz_plugins

// Tell pluginlib about this class. It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(kindr_rviz_plugins::VectorAtPositionDisplay,rviz::Display)
