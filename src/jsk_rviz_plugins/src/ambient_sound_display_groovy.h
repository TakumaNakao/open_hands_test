#ifndef __AMBIENT_SOUND_DISPLAY__
#define __AMBIENT_SOUND_DISPLAY__

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <message_filters/subscriber.hpp>
#include <tf/message_filter.hpp>
#include <jsk_rviz_plugins/msg/hark_power.hpp>
#include <ambient_sound_visual.hpp>
#include <boost/circular_buffer.hpp>
#include <rviz/message_filter_display.hpp>
#endif

#include <ros/ros.hpp>

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
    class ColorProperty;
    class FloatProperty;
    class IntProperty;
    class ROSTopicStringProperty;
}

// All the source in this plugin is in its own namespace.  This is not
// required but is good practice.
namespace jsk_rviz_plugins
{

class AmbientSoundVisual;

//class AmbientSoundDisplay: public rviz_common::Display
class AmbientSoundDisplay: public rviz_common::MessageFilterDisplay<jsk_rviz_plugins::msg::HarkPower>
{
Q_OBJECT
public:
  // Constructor.  pluginlib::ClassLoader creates instances by calling
  // the default constructor, so make sure you have one.
  AmbientSoundDisplay();
  virtual ~AmbientSoundDisplay();

  // Overrides of public virtual functions from the Display class.
  virtual void onInitialize();
  //virtual void fixedFrameChanged();
  virtual void reset();
  //virtual void createProperties();

  // Setter and getter functions for user-editable properties.
/*
  void setTopic(const std::string& topic);
  const std::string& getTopic() { return topic_; }

  void setColor( const rviz_common::properties::Color& color );
  const rviz_common::properties::Color& getColor() { return color_; }

  void setAlpha( float alpha );
  float getAlpha() { return alpha_; }

  void setHistoryLength( int history_length );
  int getHistoryLength() const { return history_length_; }

  void setWidth( float width );
  float getWidth() const { return width_; }

  void setScale( float scale );
  float getScale() const { return scale_; }

  void setBias( float bias );
  float getBias() const { return bias_; }

  void setGrad( float grad );
  float getGrad() const { return grad_; }
*/

  // Overrides of protected virtual functions from Display.  As much
  // as possible, when Displays are not enabled, they should not be
  // subscribed to incoming data and should not show anything in the
  // 3D view.  These functions are where these connections are made
  // and broken.
/*
  protected:
    virtual void onEnable();
    virtual void onDisable();
*/

private Q_SLOTS:
  void updateColorAndAlpha();
  void updateAppearance();
  void updateHistoryLength();
  bool validateFloats( const jsk_rviz_plugins::msg::HarkPower& );

  // Function to handle an incoming ROS message.
private:
  //void incomingMessage( const jsk_rviz_plugins::msg::HarkPower::ConstPtr& msg );
  void processMessage( const jsk_rviz_plugins::msg::HarkPower::ConstPtr& msg );

  // Internal helpers which do the work of subscribing and
  // unsubscribing from the ROS topic.
//  void subscribe();
//  void unsubscribe();

  // A helper to clear this display back to the initial state.
  //void clear();

  // Helper function to apply color and alpha to all visuals.

  // Storage for the list of visuals.  This display supports an
  // adjustable history length, so we need one visual per history
  // item.
  //boost::circular_buffer<AmbientSoundVisual*> visuals_;
#if ROS_VERSION_MINIMUM(1,12,0)
  boost::circular_buffer<std::shared_ptr<AmbientSoundVisual> > visuals_;
#else
  boost::circular_buffer<boost::shared_ptr<AmbientSoundVisual> > visuals_;
#endif

  // A node in the Ogre scene tree to be the parent of all our visuals.
  //Ogre::SceneNode* scene_node_;

  // Data input: Subscriber and tf message filter.
  //message_filters::Subscriber<jsk_rviz_plugins::msg::HarkPower> sub_;
  //tf::MessageFilter<jsk_rviz_plugins::msg::HarkPower>* tf_filter_;
  //int messages_received_;

  // User-editable property variables.
/*
  rviz_common::properties::Color color_;
  std::string topic_;
  float alpha_;
  int history_length_;
  float width_,scale_,bias_,grad_;
*/

  // Property objects for user-editable properties.
  rviz_common::properties::ColorProperty *color_property_;
  rviz::ROSTopicStringProperty *topic_property_;
  rviz_common::properties::FloatProperty *alpha_property_;
  rviz_common::properties::IntProperty *history_length_property_;
  rviz_common::properties::FloatProperty *width_property_;
  rviz_common::properties::FloatProperty *scale_property_;
  rviz_common::properties::FloatProperty *bias_property_;
  rviz_common::properties::FloatProperty *grad_property_;
};
// END_TUTORIAL

} // end namespace jsk_rviz_plugins

#endif // __AMBIENT_SOUND_DISPLAY__
// %EndTag(FULL_SOURCE)%
