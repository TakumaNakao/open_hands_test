#ifndef PUBLISH_TOPIC_H
#define PUBLISH_TOPIC_H

#ifndef Q_MOC_RUN
#include <ros/ros.hpp>

#include <rviz/panel.hpp>
#endif

class QLineEdit;
class QPushButton;

namespace jsk_rviz_plugins
{
  class PublishTopic: public rviz::Panel
    {
Q_OBJECT
  public:
      PublishTopic( QWidget* parent = 0 );

      virtual void load( const rviz::Config& config );
      virtual void save( rviz::Config config ) const;

      public Q_SLOTS:

      void setTopic( const QString& topic );

      protected Q_SLOTS:

      void sendVel() {}

      void updateTopic();

      void sendTopic();

    protected:
      QLineEdit* output_topic_editor_;

      QString output_topic_;

      QPushButton* send_topic_button_;

      rclcpp::Publisher<.*>::SharedPtr velocity_publisher_;

      rclcpp::Node::SharedPtr nh_;
    };

}

#endif
