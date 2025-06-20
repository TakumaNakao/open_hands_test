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

#ifndef JSK_RVIZ_PLUGINS_SIMPLE_OCCUPANCY_GRID_ARRAY_DISPLAY_H_
#define JSK_RVIZ_PLUGINS_SIMPLE_OCCUPANCY_GRID_ARRAY_DISPLAY_H_

#ifndef Q_MOC_RUN
#include <jsk_rviz_plugins/msg/simple_occupancy_grid_array.hpp>
#include <rviz/message_filter_display.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz/ogre_helpers/shape.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz/ogre_helpers/point_cloud.hpp>
#endif

namespace jsk_rviz_plugins
{
  class SimpleOccupancyGridArrayDisplay:
    public rviz_common::MessageFilterDisplay<
    jsk_rviz_plugins::msg::SimpleOccupancyGridArray>
  {
    Q_OBJECT
  public:
#if ROS_VERSION_MINIMUM(1,12,0)
    typedef std::shared_ptr<rviz_rendering::PointCloud> PointCloudPtr;
#else
    typedef boost::shared_ptr<rviz_rendering::PointCloud> PointCloudPtr;
#endif
    SimpleOccupancyGridArrayDisplay();
    virtual ~SimpleOccupancyGridArrayDisplay();
  protected:
    virtual void onInitialize();
    virtual void reset();
    virtual void allocateCloudsAndNodes(const size_t num);
    rviz_common::properties::FloatProperty* alpha_property_;
    rviz_common::properties::BoolProperty* auto_color_property_;
    double alpha_;
    std::vector<rviz_rendering::PointCloud*> clouds_;
    std::vector<Ogre::SceneNode*> nodes_;
    bool auto_color_;
  private:
    void processMessage(
      const jsk_rviz_plugins::msg::SimpleOccupancyGridArray::ConstPtr& msg);
  private Q_SLOTS:
    void updateAlpha();
    void updateAutoColor();
  };

}

#endif
