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


#ifndef JSK_RVIZ_PLUGINS_TWIST_STAMPED_H_
#define JSK_RVIZ_PLUGINS_TWIST_STAMPED_H_

#ifndef Q_MOC_RUN
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz/message_filter_display.hpp>
#include <rviz/ogre_helpers/shape.hpp>
#include <rviz/ogre_helpers/mesh_shape.hpp>
#include <rviz/ogre_helpers/arrow.hpp>
#include <OGRE/OgreSceneNode.hpp>

#include <geometry_msgs/TwistStamped.hpp>
#include <rviz/ogre_helpers/arrow.hpp>
#include <rviz/ogre_helpers/billboard_line.hpp>
#endif


namespace jsk_rviz_plugins
{
  class TwistStampedDisplay: public rviz_common::MessageFilterDisplay<geometry_msgs::TwistStamped>
  {
    Q_OBJECT
  public:
#if ROS_VERSION_MINIMUM(1,12,0)
    typedef std::shared_ptr<rviz_rendering::Arrow> ArrowPtr;
    typedef std::shared_ptr<rviz_rendering::BillboardLine> BillboardLinePtr;
#else
    typedef boost::shared_ptr<rviz_rendering::Arrow> ArrowPtr;
    typedef boost::shared_ptr<rviz_rendering::BillboardLine> BillboardLinePtr;
#endif
    TwistStampedDisplay();
    virtual ~TwistStampedDisplay();
  protected:

    virtual void onInitialize();
    virtual void reset();
    virtual void processMessage(
      const geometry_msgs::TwistStamped::ConstPtr& msg);
    virtual void updateRotationVelocity(
      BillboardLinePtr circle,
      ArrowPtr arrow,
      const Ogre::Vector3& ux,
      const Ogre::Vector3& uy,
      const Ogre::Vector3& uz,
      const double r,
      bool positive);
    ////////////////////////////////////////////////////////
    // properties
    ////////////////////////////////////////////////////////
    rviz_common::properties::FloatProperty* linear_scale_property_;
    rviz_common::properties::FloatProperty* angular_scale_property_;
    rviz_common::properties::ColorProperty* linear_color_property_;
    rviz_common::properties::ColorProperty* angular_color_property_;
    
    double linear_scale_;
    double angular_scale_;
    QColor linear_color_;
    QColor angular_color_;
                         
    ArrowPtr linear_arrow_;
    BillboardLinePtr x_rotate_circle_;
    BillboardLinePtr y_rotate_circle_;
    BillboardLinePtr z_rotate_circle_;
    ArrowPtr x_rotate_arrow_;
    ArrowPtr y_rotate_arrow_;
    ArrowPtr z_rotate_arrow_;
  private Q_SLOTS:
    void updateLinearScale();
    void updateAngularScale();
    void updateLinearColor();
    void updateAngularColor();
  };
}

#endif
