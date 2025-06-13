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
#ifndef JSK_RVIZ_PLUGIN_OVERLAY_DIAGNOSTIC_DISPLAY_H_
#define JSK_RVIZ_PLUGIN_OVERLAY_DIAGNOSTIC_DISPLAY_H_

#ifndef Q_MOC_RUN
#include <rviz_common/display.hpp>
#include <OGRE/OgreTexture.hpp>
#include <OGRE/OgreColourValue.hpp>
#include <OGRE/OgreMaterial.hpp>

#include <QPainter>
#include <QPainterPath>

#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/editable_enum_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/color_property.hpp>

#include <diagnostic_msgs/DiagnosticArray.hpp>

#include "overlay_utils.h"
#endif

namespace jsk_rviz_plugins
{
  class OverlayDiagnosticDisplay: public rviz_common::Display
  {
    Q_OBJECT
  public:
    OverlayDiagnosticDisplay();
    typedef enum {OK_STATE, ERROR_STATE, WARN_STATE, STALL_STATE} State;
    virtual ~OverlayDiagnosticDisplay();

    // methods for OverlayPickerTool
    virtual bool isInRegion(int x, int y);
    virtual void movePosition(int x, int y);
    virtual void setPosition(int x, int y);
    virtual int getX() { return left_; };
    virtual int getY() { return top_; };

  protected:
    virtual bool isStalled();
    virtual void processMessage(
      const diagnostic_msgs::DiagnosticArray::ConstPtr& msg);
    virtual void update(float wall_dt, float ros_dt);
    virtual void onEnable();
    virtual void onDisable();
    virtual void onInitialize();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void redraw();
    virtual State getLatestState();
    virtual void drawSAC(QImage& Hud);
    virtual void drawEVA(QImage& Hud);
    virtual void drawEVAConnectedRectangle(QPainter& painter,
                                           QColor color,
                                           QColor small_color,
                                           int width);
    virtual void drawEVANonConnectedRectangle(QPainter& painter,
                                              QColor color,
                                              QColor small_color,
                                              int width,
                                              double gap);
    virtual void fillNamespaceList();
    virtual QColor foregroundColor();
    virtual QColor textColor();
    virtual double textWidth(QPainter& painter, double font_size, const std::string& text);
    virtual double textHeight(QPainter& painter, double font_size);
    virtual QColor blendColor(QColor a, QColor b, double a_rate);
    virtual void drawText(QPainter& painter, QColor fg_color,
                          const std::string& text);
    virtual double drawAnimatingText(QPainter& painter,
                                     QColor fg_color,
                                     const double height,
                                     const double font_size,
                                     const std::string text);
    // return true if plugin needs to animate
    virtual bool isAnimating();
    virtual double animationRate();
    virtual std::string statusText();
    boost::mutex mutex_;
    OverlayObject::Ptr overlay_;
    
#if ROS_VERSION_MINIMUM(1,12,0)
    std::shared_ptr<diagnostic_msgs::DiagnosticStatus> latest_status_;
#else
    boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> latest_status_;
#endif
    State previous_state_;
    rclcpp::Time latest_message_time_;
    rclcpp::Time animation_start_time_;
    int size_;
    std::string diagnostics_namespace_;
    int type_;
    std::set<std::string> namespaces_;
    double alpha_;
    int top_, left_;
    double t_;
    double stall_duration_;
    bool is_animating_;
    rviz_common::properties::RosTopicProperty* ros_topic_property_;
    rviz::EditableEnumProperty* diagnostics_namespace_property_;
    rviz_common::properties::EnumProperty* type_property_;
    rviz_common::properties::IntProperty* top_property_;
    rviz_common::properties::IntProperty* left_property_;
    rviz_common::properties::FloatProperty* alpha_property_;
    rviz_common::properties::IntProperty* size_property_;
    rviz_common::properties::FloatProperty* stall_duration_property_;
    
    rclcpp::Subscription<.*>::SharedPtr sub_;
  protected Q_SLOTS:
    virtual void updateType();
    virtual void updateRosTopic();
    virtual void updateDiagnosticsNamespace();
    virtual void updateSize();
    virtual void updateAlpha();
    virtual void updateTop();
    virtual void updateLeft();
    virtual void updateStallDuration();
  private:
    
  };
}

#endif
