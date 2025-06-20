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

#ifndef JSK_RVIZ_PLUGINS_SEGMENT_ARRAY_DISPLAY_H_
#define JSK_RVIZ_PLUGINS_SEGMENT_ARRAY_DISPLAY_H_

#ifndef Q_MOC_RUN
#include <jsk_rviz_plugins/msg/segment_array.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz/message_filter_display.hpp>
#include <rviz/ogre_helpers/shape.hpp>
#include <rviz/ogre_helpers/billboard_line.hpp>
#include <rviz/ogre_helpers/arrow.hpp>
#include <OGRE/OgreSceneManager.hpp>
#include <OGRE/OgreSceneNode.hpp>
#endif

namespace jsk_rviz_plugins
{
  class SegmentArrayDisplay:
    public rviz_common::MessageFilterDisplay<jsk_rviz_plugins::msg::SegmentArray>
  {
    Q_OBJECT
  public:
#if ROS_VERSION_MINIMUM(1,12,0)
    typedef std::shared_ptr<rviz_rendering::BillboardLine> BillboardLinePtr;
#else
    typedef boost::shared_ptr<rviz_rendering::BillboardLine> BillboardLinePtr;
#endif
    SegmentArrayDisplay();
    virtual ~SegmentArrayDisplay();
  protected:
    virtual void onInitialize();
    virtual void reset();
    void allocateBillboardLines(int num);
    QColor getColor(size_t index);
    virtual void showEdges(
      const jsk_rviz_plugins::msg::SegmentArray::ConstPtr& msg);

    rviz_common::properties::EnumProperty* coloring_property_;
    rviz_common::properties::ColorProperty* color_property_;
    rviz_common::properties::FloatProperty* alpha_property_;
    rviz_common::properties::FloatProperty* line_width_property_;
    QColor color_;
    double alpha_;
    std::string coloring_method_;
    double line_width_;
    std::vector<BillboardLinePtr> edges_;

    jsk_rviz_plugins::msg::SegmentArray::ConstPtr latest_msg_;
  private Q_SLOTS:
    void updateColor();
    void updateAlpha();
    void updateColoring();
    void updateLineWidth();
  private:
    void processMessage(
      const jsk_rviz_plugins::msg::SegmentArray::ConstPtr& msg);
  };

}
#endif
