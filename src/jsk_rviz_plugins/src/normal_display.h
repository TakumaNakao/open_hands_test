// -*- mode: C++ -*-
#ifndef NORMAL_DISPLAY_H
#define NORMAL_DISPLAY_H
#ifndef Q_MOC_RUN
#include <OGRE/OgreSceneNode.hpp>
#include <OGRE/OgreSceneManager.hpp>
#include <QColor>

#include <boost/circular_buffer.hpp>

#include <sensor_msgs/PointCloud2.hpp>

#include <rviz/message_filter_display.hpp>
#include <rviz/default_plugin/point_cloud_transformers.hpp>
#include <rviz/validate_floats.hpp>
#include <rviz/visualization_manager.hpp>
#include <rviz/frame_manager.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include "normal_visual.h"
#endif

namespace jsk_rviz_plugins
{

class NormalDisplay: public rviz_common::MessageFilterDisplay<sensor_msgs::PointCloud2>
{
Q_OBJECT
public:
  NormalDisplay();
  virtual ~NormalDisplay();
  rviz_common::properties::EnumProperty* style_property_;
  rviz_common::properties::ColorProperty* color_property_;
  rviz_common::properties::ColorProperty* min_color_property_;
  rviz_common::properties::ColorProperty* max_color_property_;
  rviz_common::properties::FloatProperty* skip_rate_property_;
  rviz_common::properties::BoolProperty* rainbow_property_;
  rviz_common::properties::FloatProperty* scale_property_;
  rviz_common::properties::FloatProperty* alpha_property_;
  float skip_rate_;
  float scale_;
  float alpha_;

  enum ColorTypes{
    POINTS_COLOR,
    FLAT_COLOR,
    DIRECTION_COLOR,
    CURVATURE_COLOR
  };

protected:
  virtual void onInitialize();

  virtual void reset();

#if ROS_VERSION_MINIMUM(1,12,0)
  boost::circular_buffer<std::shared_ptr<NormalVisual> > visuals_;
#else
  boost::circular_buffer<boost::shared_ptr<NormalVisual> > visuals_;
#endif


  // Function to handle an incoming ROS message.
private Q_SLOTS:
  void processMessage( const sensor_msgs::PointCloud2::ConstPtr& msg );
  void updateStyle();
  void updateSkipRate();
  void updateRainbow();
  void updateScale();
  void updateAlpha();
  void getRainbow(float value , float& rf, float& gf, float& bf);
};

}

#endif
