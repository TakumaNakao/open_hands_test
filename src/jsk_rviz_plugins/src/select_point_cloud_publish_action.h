#ifndef SELECT_POINT_CLOUD_PUBLISH_ACTION_H
#define SELECT_POINT_CLOUD_PUBLISH_ACTION_H

#ifndef Q_MOC_RUN
#include <ros/ros.hpp>

#include <rviz/panel.hpp>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#  include <QtWidgets>
#else
#  include <QtGui>
#endif
#endif

class QLineEdit;
class QLabel;
class QPushButton;
//class QSignalMapper;
class PropertyTreeWidget;


namespace jsk_rviz_plugins
{
  class SelectPointCloudPublishAction: public rviz_common::Panel
    {
      // This class uses Qt slots and is a subclass of QObject, so it needs
      // the Q_OBJECT macro.
Q_OBJECT
  public:
      SelectPointCloudPublishAction( QWidget* parent = 0 );

      virtual void load( const rviz_common::Config& config );
      virtual void save( rviz_common::Config config ) const;

      protected Q_SLOTS:

      void publishPointCloud();
    protected:
      QPushButton* publish_pointcloud_button_;

      QVBoxLayout* layout;

      // The ROS publisher for the command velocity.
      rclcpp::Publisher<.*>::SharedPtr select_pointcloud_publisher_;

      // The ROS node handle.
      rclcpp::Node::SharedPtr nh_;

    };

}

#endif // TELEOP_PANEL_H
