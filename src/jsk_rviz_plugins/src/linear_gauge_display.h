// -*- mode: c++; -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
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
#ifndef JSK_RVIZ_PLUGIN_LINEAR_GAUGE_H_
#define JSK_RVIZ_PLUGIN_LINEAR_GAUGE_H_

#include "std_msgs/msg/msg/float32.hpp"
#ifndef Q_MOC_RUN
#include <rviz_common/display.hpp>
#include "overlay_utils.h"
#include <OGRE/OgreColourValue.hpp>
#include <OGRE/OgreTexture.hpp>
#include <OGRE/OgreMaterial.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosidl_runtime_cpp/traits.hpp>
#endif

namespace jsk_rviz_plugins
{

  class LinearGaugeDisplay
    : public rviz_common::Display
  {
    Q_OBJECT
  public:
    LinearGaugeDisplay();
    virtual ~LinearGaugeDisplay();
    // methods for OverlayPickerTool
    virtual bool isInRegion(int x, int y);
    virtual void movePosition(int x, int y);
    virtual void setPosition(int x, int y);
    virtual int getX() { return left_; };
    virtual int getY() { return top_; };
  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void update(float wall_dt, float ros_dt);
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void onEnable();
    virtual void onDisable();
    virtual void onInitialize();
    virtual void processMessage(const std_msgs::Float32::ConstPtr& msg);
    virtual void drawPlot();
    ////////////////////////////////////////////////////////
    // properties
    ////////////////////////////////////////////////////////
    rviz_common::properties::RosTopicProperty* update_topic_property_;
    rviz_common::properties::BoolProperty* show_value_property_;
    rviz_common::properties::BoolProperty* vertical_gauge_property_;
    rviz_common::properties::ColorProperty* fg_color_property_;
    rviz_common::properties::ColorProperty* bg_color_property_;
    rviz_common::properties::FloatProperty* fg_alpha_property_;
    rviz_common::properties::FloatProperty* bg_alpha_property_;
    rviz_common::properties::FloatProperty* update_interval_property_;
    rviz_common::properties::BoolProperty* show_border_property_;
    rviz_common::properties::IntProperty* buffer_length_property_;
    rviz_common::properties::IntProperty* width_property_;
    rviz_common::properties::IntProperty* height_property_;
    rviz_common::properties::IntProperty* left_property_;
    rviz_common::properties::IntProperty* top_property_;
    rviz_common::properties::IntProperty* line_width_property_;
    rviz_common::properties::BoolProperty* auto_color_change_property_;
    rviz_common::properties::ColorProperty* max_color_property_;
    rviz_common::properties::BoolProperty* show_caption_property_;
    rviz_common::properties::IntProperty* text_size_property_;
    rviz_common::properties::FloatProperty* max_value_property_;
    rviz_common::properties::FloatProperty* min_value_property_;
    
    OverlayObject::Ptr overlay_;
    QColor fg_color_;
    QColor max_color_;
    QColor bg_color_;
   
    double fg_alpha_;
    double bg_alpha_;
    bool show_border_;
    bool auto_color_change_;
    bool show_value_;
    bool show_caption_;
    bool vertical_gauge_;
    bool draw_required_;
    float last_time_;
    float update_interval_;
    
    uint16_t texture_width_;
    uint16_t texture_height_;
    int left_;
    int top_;
    int line_width_;
    int text_size_;
    int caption_offset_;
    double min_value_;
    double max_value_;
    const int width_padding_;
    const int height_padding_;

    float data_;
    bool first_time_;

    
    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    boost::mutex mutex_;
    rclcpp::Subscription<.*>::SharedPtr sub_;
                        
  protected Q_SLOTS:
    void updateTopic();
    void updateShowValue();
    void updateVerticalGauge();
    void updateBGColor();
    void updateFGColor();
    void updateFGAlpha();
    void updateBGAlpha();
    void updateWidth();
    void updateHeight();
    void updateTop();
    void updateLeft();
    void updateLineWidth();
    void updateShowBorder();
    void updateAutoColorChange();
    void updateMaxColor();
    void updateUpdateInterval();
    void updateShowCaption();
    void updateTextSize();
    void updateMinValue();
    void updateMaxValue();
  private:
  };
}

#endif
