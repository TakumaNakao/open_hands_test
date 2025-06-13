#include "yes_no_button_interface.h"
#include <mutex>
#include <rviz_common/config.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSignalMapper>

// TODO: ROS2 - Replace with ROS2 equivalent or create custom service
// #include <jsk_gui_msgs/srv/yes_no.hpp>
#include <std_srvs/srv/trigger.hpp>


namespace jsk_rviz_plugins
{

  YesNoButtonInterface::YesNoButtonInterface(QWidget* parent)
    : rviz_common::Panel(parent)
  {
    layout_ = new QHBoxLayout;

    yes_button_ = new QPushButton("Yes");
    layout_->addWidget(yes_button_);
    yes_button_->setEnabled(false);

    no_button_ = new QPushButton("No");
    layout_->addWidget(no_button_);
    no_button_->setEnabled(false);

    connect(yes_button_, SIGNAL(clicked()), this, SLOT(respondYes()));
    connect(no_button_, SIGNAL(clicked()), this, SLOT(respondNo()));

    setLayout(layout_);
  }

  void YesNoButtonInterface::onInitialize()
  {
    // TODO: ROS2 - Replace with proper service server
    // auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
    // yes_no_button_service_ = node->create_service<std_srvs::srv::Trigger>(
    //   "/rviz/yes_no_button",
    //   std::bind(&YesNoButtonInterface::requested, this, std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(rclcpp::get_logger("yes_no_button_interface"), "YesNo button interface initialized");
  }

  // TODO: ROS2 - Replace with proper service callback signature
  void YesNoButtonInterface::requested(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    need_user_input_ = true;
    yes_button_->setEnabled(true);
    no_button_->setEnabled(true);
    while (need_user_input_) {
      QApplication::processEvents(QEventLoop::AllEvents, 100);
    }
    yes_button_->setEnabled(false);
    no_button_->setEnabled(false);
    response->success = yes_;
    response->message = yes_ ? "Yes" : "No";
  }

  void YesNoButtonInterface::respondYes()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    yes_ = true;
    need_user_input_ = false;
  }

  void YesNoButtonInterface::respondNo()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    yes_ = false;
    need_user_input_ = false;
  }

  void YesNoButtonInterface::save(rviz_common::Config config) const
  {
    rviz_common::Panel::save(config);
  }

  void YesNoButtonInterface::load(const rviz_common::Config& config)
  {
    rviz_common::Panel::load(config);
  }

}  // namespace jsk_rviz_plugins


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::YesNoButtonInterface, rviz_common::Panel)
