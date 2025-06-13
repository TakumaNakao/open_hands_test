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

#ifndef FOOTSTEP_DISPLAY_H
#define FOOTSTEP_DISPLAY_H

#include <jsk_footstep_msgs/FootstepArray.hpp>
#ifndef Q_MOC_RUN
#include <rviz/message_filter_display.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz/ogre_helpers/billboard_line.hpp>
#include <rviz/ogre_helpers/shape.hpp>
#include <rviz/ogre_helpers/movable_text.hpp>
#include <OGRE/OgreSceneManager.hpp>
#include <OGRE/OgreSceneNode.hpp>
#endif

namespace jsk_rviz_plugins
{
  class FootstepDisplay : public rviz_common::MessageFilterDisplay<jsk_footstep_msgs::FootstepArray>
  {
    Q_OBJECT
  public:
    FootstepDisplay();
    virtual ~FootstepDisplay();
  protected:
    virtual void onInitialize();
    virtual void reset();
    virtual void update(float wall_dt, float ros_dt);
  private:
    virtual void allocateCubes(size_t num);
    virtual void allocateTexts(size_t num);
    virtual double estimateTextSize(
      const jsk_footstep_msgs::Footstep& footstep);
    virtual double minNotZero(double a, double b);
    virtual void processMessage(const jsk_footstep_msgs::FootstepArray::ConstPtr& msg);
    virtual bool validateFloats(const jsk_footstep_msgs::FootstepArray& msg);
    
    rviz_common::properties::FloatProperty* alpha_property_;
    rviz_common::properties::FloatProperty* width_property_;
    rviz_common::properties::FloatProperty* height_property_;
    rviz_common::properties::FloatProperty* depth_property_;
    rviz_common::properties::BoolProperty* show_name_property_;
    rviz_common::properties::BoolProperty* use_group_coloring_property_;
    jsk_footstep_msgs::FootstepArray::ConstPtr latest_footstep_;
#if ROS_VERSION_MINIMUM(1,12,0)
    typedef std::shared_ptr<rviz_rendering::Shape> ShapePtr;
#else
    typedef boost::shared_ptr<rviz_rendering::Shape> ShapePtr;
#endif
    std::vector<ShapePtr> shapes_;
    std::vector<rviz_rendering::MovableText*> texts_;
    std::vector<Ogre::SceneNode*> text_nodes_;
    rviz_rendering::BillboardLine* line_;
    double width_, height_, depth_;
    double alpha_;
    bool show_name_;
    bool use_group_coloring_;
    //Ogre::SceneNode* scene_node_;
  private Q_SLOTS:
    void updateAlpha();
    void updateWidth();
    void updateHeight();
    void updateDepth();
    void updateShowName();
    void updateUseGroupColoring();
  };
}

#endif
