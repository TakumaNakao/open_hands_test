#include "rviz_common/config.hpp"
#include "empty_service_call_interface.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSignalMapper>

using namespace rviz_common;
namespace jsk_rviz_plugins
{
  EmptyServiceCallInterfaceAction::EmptyServiceCallInterfaceAction( QWidget* parent )
    : rviz_common::Panel( parent )
  {
    parseROSParameters();

    QHBoxLayout* h_layout = new QHBoxLayout;
    h_layout->setAlignment(Qt::AlignLeft);
    layout = new QVBoxLayout();
    signal_mapper = new QSignalMapper(this);

    for(size_t i = 0; i < service_call_button_infos_.size();i++){
      ServiceCallButtonInfo target_button = service_call_button_infos_[i];
      QToolButton* tbutton = new QToolButton(this);
      tbutton->setText(target_button.text.c_str());
      tbutton->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
      tbutton->setIconSize(QSize(100, 100));
      tbutton->setIcon(QIcon(QPixmap(QString(target_button.icon_file_path.c_str()))));
      connect(tbutton, SIGNAL(clicked()), signal_mapper, SLOT(map()));
      signal_mapper->setMapping(tbutton, i);
      h_layout->addWidget(tbutton);
    };
    connect(signal_mapper, SIGNAL(mapped(int)),
            this, SLOT(callRequestEmptyCommand(int)));
    layout->addLayout(h_layout);
    setLayout( layout );
  }

  void EmptyServiceCallInterfaceAction::parseROSParameters(){
    // TODO: ROS2 - Replace with proper parameter handling
    // For now, add default buttons
    
    std::string icon_package_name = "jsk_rviz_plugins";
    RCLCPP_INFO(rclcpp::get_logger("empty_service_call_interface"), "Find Icons In %s package.", icon_package_name.c_str());

    std::string icon_path_prefix;
    try {
      icon_path_prefix = ament_index_cpp::get_package_share_directory(icon_package_name) + "/icons/";
    } catch (const std::exception& e) {
      RCLCPP_WARN(rclcpp::get_logger("empty_service_call_interface"), "Package %s not found, using default path", icon_package_name.c_str());
      icon_path_prefix = "/tmp/";
    }

    // TODO: ROS2 - Replace with parameter client
    // For now, add default service buttons
    ServiceCallButtonInfo default_button;
    default_button.icon_file_path = icon_path_prefix + "default.png";
    default_button.service_name = "/empty_service";
    default_button.text = "Call Service";
    service_call_button_infos_.push_back(default_button);
  };

  void EmptyServiceCallInterfaceAction::callRequestEmptyCommand(const int button_id){
    // TODO: ROS2 - Replace with proper service client
    // auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
    // auto client = node->create_client<std_srvs::srv::Empty>(service_call_button_infos_[button_id].service_name);
    // auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    // auto future = client->async_send_request(request);
    
    RCLCPP_INFO(rclcpp::get_logger("empty_service_call_interface"), "Service call requested for: %s", 
                service_call_button_infos_[button_id].service_name.c_str());
  }

  void EmptyServiceCallInterfaceAction::save( rviz_common::Config config ) const
  {
    rviz_common::Panel::save( config );
  }

  void EmptyServiceCallInterfaceAction::load( const rviz_common::Config& config )
  {
    rviz_common::Panel::load( config );
  }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::EmptyServiceCallInterfaceAction, rviz_common::Panel )
