/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ryohei Ueda and JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "bounding_box_display_common.h"
#include "bounding_box_display.h"
#include <jsk_topic_tools/color_utils.h>

namespace jsk_rviz_plugins
{

  BoundingBoxDisplay::BoundingBoxDisplay()
  {
    coloring_property_ = new rviz_common::properties::EnumProperty(
      "coloring", "Flat color",
      "coloring method",
      this, SLOT(updateColoring()));
    coloring_property_->addOption("Flat color", 0);
    coloring_property_->addOption("Label", 1);
    coloring_property_->addOption("Value", 2);

    alpha_method_property_ = new rviz_common::properties::EnumProperty(
      "alpha_method", "flat", "alpha method",
      this, SLOT(updateAlphaMethod()));
    alpha_method_property_->addOption("flat", 0);
    alpha_method_property_->addOption("value", 1);

    color_property_ = new rviz_common::properties::ColorProperty(
      "color", QColor(25, 255, 0),
      "color to draw the bounding boxes",
      this, SLOT(updateColor()));
    alpha_property_ = new rviz_common::properties::FloatProperty(
      "alpha", 0.8,
      "alpha value to draw the bounding boxes",
      this, SLOT(updateAlpha()));
    alpha_min_property_ = new rviz_common::properties::FloatProperty(
      "alpha min", 0.0,
      "alpha value corresponding to value = 0",
      this, SLOT(updateAlphaMin()));
    alpha_max_property_ = new rviz_common::properties::FloatProperty(
      "alpha max", 1.0,
      "alpha value corresponding to value = 1",
      this, SLOT(updateAlphaMax()));
    only_edge_property_ = new rviz_common::properties::BoolProperty(
      "only edge", false,
      "show only the edges of the boxes",
      this, SLOT(updateOnlyEdge()));
    line_width_property_ = new rviz_common::properties::FloatProperty(
      "line width", 0.005,
      "line width of the edges",
      this, SLOT(updateLineWidth()));
    show_coords_property_ = new rviz_common::properties::BoolProperty(
      "show coords", false,
      "show coordinate of bounding box",
      this, SLOT(updateShowCoords()));
    value_threshold_property_ = new rviz_common::properties::FloatProperty(
      "value threshold", 0.0,
      "filter all boxes with value < threshold",
      this, SLOT(updateValueThreshold()));
  }

  BoundingBoxDisplay::~BoundingBoxDisplay()
  {
    delete color_property_;
    delete alpha_property_;
    delete alpha_min_property_;
    delete alpha_max_property_;
    delete only_edge_property_;
    delete coloring_property_;
    delete alpha_method_property_;
    delete show_coords_property_;
    delete value_threshold_property_;
  }

  void BoundingBoxDisplay::onInitialize()
  {
    MFDClass::onInitialize();
    scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

    updateColor();
    updateAlpha();
    updateAlphaMin();
    updateAlphaMax();
    updateOnlyEdge();
    updateColoring();
    updateAlphaMethod();
    updateLineWidth();
    updateShowCoords();
    updateValueThreshold();
  }

  void BoundingBoxDisplay::updateLineWidth()
  {
    line_width_ = line_width_property_->getFloat();
    if (latest_msg_) {
      processMessage(latest_msg_);
    }
  }

  void BoundingBoxDisplay::updateColor()
  {
    color_ = color_property_->getColor();
    if (latest_msg_) {
      processMessage(latest_msg_);
    }
  }

  void BoundingBoxDisplay::updateAlpha()
  {
    alpha_ = alpha_property_->getFloat();
    if (latest_msg_) {
      processMessage(latest_msg_);
    }
  }

  void BoundingBoxDisplay::updateAlphaMin()
  {
    if (alpha_min_property_->getFloat() > alpha_max_)
    {
      ROS_WARN("alpha_min must be <= alpha_max");
      alpha_min_property_->setFloat(alpha_min_);
      return;
    }
    alpha_min_ = alpha_min_property_->getFloat();
    if (latest_msg_) {
      processMessage(latest_msg_);
    }
  }

  void BoundingBoxDisplay::updateAlphaMax()
  {
    if (alpha_max_property_->getFloat() < alpha_min_)
    {
      ROS_WARN("alpha_min must be <= alpha_max");
      alpha_max_property_->setFloat(alpha_max_);
      return;
    }
    alpha_max_ = alpha_max_property_->getFloat();
    if (latest_msg_) {
      processMessage(latest_msg_);
    }
  }

  void BoundingBoxDisplay::updateOnlyEdge()
  {
    only_edge_ = only_edge_property_->getBool();
    if (only_edge_) {
      line_width_property_->show();
    }
    else {
      line_width_property_->hide();;
    }
    if (latest_msg_) {
      processMessage(latest_msg_);
    }
  }

  void BoundingBoxDisplay::updateColoring()
  {
    if (coloring_property_->getOptionInt() == 0) {
      coloring_method_ = "flat";
      color_property_->show();
    }
    else if (coloring_property_->getOptionInt() == 1) {
      coloring_method_ = "label";
      color_property_->hide();
    }
    else if (coloring_property_->getOptionInt() == 2) {
      coloring_method_ = "value";
      color_property_->hide();
    }

    if (latest_msg_) {
      processMessage(latest_msg_);
    }
  }

  void BoundingBoxDisplay::updateAlphaMethod()
  {
    if (alpha_method_property_->getOptionInt() == 0) {
      alpha_method_ = "flat";
      alpha_property_->show();
      alpha_min_property_->hide();
      alpha_max_property_->hide();
    }
    else if (alpha_method_property_->getOptionInt() == 1) {
      alpha_method_ = "value";
      alpha_property_->hide();
      alpha_min_property_->show();
      alpha_max_property_->show();
    }

    if (latest_msg_) {
      processMessage(latest_msg_);
    }
  }

  void BoundingBoxDisplay::updateShowCoords()
  {
    show_coords_ = show_coords_property_->getBool();
    // Immediately apply show_coords attribute
    if (!show_coords_) {
      hideCoords();
    }
    else if (latest_msg_) {
      processMessage(latest_msg_);
    }
  }

  void BoundingBoxDisplay::reset()
  {
    MFDClass::reset();
    shapes_.clear();
    edges_.clear();
    coords_nodes_.clear();
    coords_objects_.clear();
    latest_msg_.reset();
  }

  void BoundingBoxDisplay::processMessage(
    const jsk_rviz_plugins::msg::BoundingBox::ConstPtr& msg)
  {
    // Store latest message
    latest_msg_ = msg;

    // Convert bbox to bbox_array to show it
    jsk_rviz_plugins::msg::BoundingBoxArrayPtr bbox_array_msg(new jsk_rviz_plugins::msg::BoundingBoxArray);
    bbox_array_msg->header = msg->header;
    std::vector<jsk_rviz_plugins::msg::BoundingBox> boxes;
    boxes.push_back(*msg);
    bbox_array_msg->boxes = boxes;

    if (!only_edge_) {
      showBoxes(bbox_array_msg);
    }
    else {
      showEdges(bbox_array_msg);
    }

    if (show_coords_) {
      showCoords(bbox_array_msg);
    }
    else {
      hideCoords();
    }
  }

  void BoundingBoxDisplay::updateValueThreshold()
  {
    if (value_threshold_property_->getFloat() < 0.0 || value_threshold_property_->getFloat() > 1.0)
    {
      ROS_WARN("value threshold must be in [0,1]");
      value_threshold_property_->setFloat(value_threshold_);
      return;
    }
    value_threshold_ = value_threshold_property_->getFloat();
    if (latest_msg_) {
      processMessage(latest_msg_);
    }
  }

}  // namespace jsk_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::BoundingBoxDisplay, rviz_common::Display)
