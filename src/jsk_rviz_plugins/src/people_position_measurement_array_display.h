// -*- mode: c++ -*-
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


#ifndef JSK_RVIZ_PLUGIN_PEOPLE_POSITION_MEASUREMENT_ARRAY_DISPLAY_H_
#define JSK_RVIZ_PLUGIN_PEOPLE_POSITION_MEASUREMENT_ARRAY_DISPLAY_H_


#ifndef Q_MOC_RUN
#include <rviz_common/display.hpp>
#include <rviz/message_filter_display.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/editable_enum_property.hpp>
#include <rviz_common/properties/tf_frame_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz/display_context.hpp>
#include <rviz/frame_manager.hpp>
#include <people_msgs/PositionMeasurementArray.hpp>
#include "overlay_utils.h"
#include "facing_visualizer.h"
#endif

namespace jsk_rviz_plugins
{
  
  class PeoplePositionMeasurementArrayDisplay:
    public rviz_common::MessageFilterDisplay<people_msgs::PositionMeasurementArray>
  {
    Q_OBJECT
  public:
    PeoplePositionMeasurementArrayDisplay();
    virtual ~PeoplePositionMeasurementArrayDisplay();
  protected:
    virtual void onInitialize();
    virtual void reset();
    void processMessage(const people_msgs::PositionMeasurementArray::ConstPtr& msg);
    void update(float wall_dt, float ros_dt);
    void clearObjects();
    rviz_common::properties::FloatProperty* size_property_;
    rviz_common::properties::FloatProperty* timeout_property_;
    rviz_common::properties::BoolProperty* anonymous_property_;
    rviz_common::properties::StringProperty* text_property_;
    boost::mutex mutex_;
    double size_;
    double timeout_;
    bool anonymous_;
    std::string text_;
    std::vector<people_msgs::PositionMeasurement> faces_;
    std::vector<GISCircleVisualizer::Ptr> visualizers_;
    ros::Time latest_time_;
  private Q_SLOTS:
    void updateSize();
    void updateTimeout();
    void updateAnonymous();
    void updateText();
  private:
  
    
  };
}

#endif
