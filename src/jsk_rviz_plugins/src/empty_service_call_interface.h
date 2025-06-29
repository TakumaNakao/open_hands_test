#ifndef EMPTY_SERCICE_CALL_INTERFACE_H
#define EMPTY_SERCICE_CALL_INTERFACE_H

#ifndef Q_MOC_RUN
#include <ros/ros.hpp>
#include <rviz/panel.hpp>
#include <std_srvs/Empty.hpp>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#  include <QtWidgets>
#else
#  include <QtGui>
#endif
#endif

namespace jsk_rviz_plugins
{
  struct ServiceCallButtonInfo
  {
    std::string icon_file_path;
    std::string service_name;
    std::string text;
  };

  class EmptyServiceCallInterfaceAction: public rviz_common::Panel
  {
    Q_OBJECT
    public:
    EmptyServiceCallInterfaceAction( QWidget* parent = 0 );

    virtual void load( const rviz_common::Config& config );
    virtual void save( rviz_common::Config config ) const;

  protected Q_SLOTS:
    void callRequestEmptyCommand(int button_id);
    void parseROSParameters();
  protected:
    rclcpp::Node::SharedPtr nh_;
    std::vector<ServiceCallButtonInfo> service_call_button_infos_;
    QVBoxLayout* layout;
    QHBoxLayout* h_layout;
    QSignalMapper* signal_mapper;
  };
}

#endif
