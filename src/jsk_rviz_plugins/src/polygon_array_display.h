// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
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

#ifndef POLYGON_ARRAY_DISPLAY_H
#define POLYGON_ARRAY_DISPLAY_H

#ifndef Q_MOC_RUN
#include <jsk_rviz_plugins/msg/polygon_array.hpp>
#include <rviz/message_filter_display.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz/ogre_helpers/billboard_line.hpp>
#include <rviz/ogre_helpers/shape.hpp>

#include <OGRE/OgreSceneManager.hpp>
#include <OGRE/OgreSceneNode.hpp>
#include <OGRE/OgreManualObject.hpp>
#include <OGRE/OgreMaterialManager.hpp>
#include <OGRE/OgreTechnique.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz/ogre_helpers/billboard_line.hpp>
#include <rviz/ogre_helpers/arrow.hpp>
#endif

#include <ros/ros.hpp>

namespace jsk_rviz_plugins
{
  
  class PolygonArrayDisplay:
    public rviz_common::MessageFilterDisplay<jsk_rviz_plugins::msg::PolygonArray>
  {
    Q_OBJECT
  public:
#if ROS_VERSION_MINIMUM(1,12,0)
    typedef std::shared_ptr<rviz_rendering::Arrow> ArrowPtr;
#else
    typedef boost::shared_ptr<rviz_rendering::Arrow> ArrowPtr;
#endif
    PolygonArrayDisplay();
    virtual ~PolygonArrayDisplay();
  protected:
    virtual void onInitialize();
    virtual void reset();
    virtual void updateSceneNodes(
      const jsk_rviz_plugins::msg::PolygonArray::ConstPtr& msg);
    virtual void allocateMaterials(int num);
    virtual void updateLines(int num);
    virtual Ogre::ColourValue getColor(size_t index);
    virtual void processLine(
      const size_t i, const geometry_msgs::PolygonStamped& polygon);
    virtual void processPolygon(
      const size_t i, const geometry_msgs::PolygonStamped& polygon);
    virtual void processNormal(
      const size_t i, const geometry_msgs::PolygonStamped& polygon);
    virtual void processPolygonMaterial(const size_t i);
    virtual void processMessage(
      const jsk_rviz_plugins::msg::PolygonArray::ConstPtr& msg);
    virtual bool getTransform(
      const std_msgs::Header &header,
      Ogre::Vector3& position, Ogre::Quaternion& orientation);
    rviz_common::properties::ColorProperty* color_property_;
    rviz_common::properties::FloatProperty* alpha_property_;
    rviz_common::properties::BoolProperty* only_border_property_;
    // rviz_common::properties::BoolProperty* auto_coloring_property_;
    rviz_common::properties::EnumProperty* coloring_property_;
    rviz_common::properties::BoolProperty* show_normal_property_;
    rviz_common::properties::BoolProperty* enable_lighting_property_;
    rviz_common::properties::FloatProperty* normal_length_property_;
    bool only_border_;
    bool enable_lighting_;
    std::string coloring_method_;
    bool show_normal_;
    double normal_length_;
    jsk_rviz_plugins::msg::PolygonArray::ConstPtr latest_msg_;
    std::vector<Ogre::ManualObject*> manual_objects_;
    std::vector<Ogre::SceneNode*> scene_nodes_;
    std::vector<Ogre::SceneNode*> arrow_nodes_;
    std::vector<ArrowPtr> arrow_objects_;
    std::vector<Ogre::MaterialPtr> materials_;
    std::vector<rviz_rendering::BillboardLine*> lines_;
  private Q_SLOTS:
    void updateColoring();
    void updateOnlyBorder();
    void updateShowNormal();
    void updateEnableLighting();
    void updateNormalLength();
  private:
    
  };
}

#endif

