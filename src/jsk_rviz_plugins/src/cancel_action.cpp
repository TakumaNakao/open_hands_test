#include <stdio.hpp>

#include <QPainter>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <std_msgs/msg/msg/empty.hpp>
#include <action_msgs/msg/goal_info.hpp>

#include "cancel_action.h"

namespace jsk_rviz_plugins
{

  CancelAction::CancelAction( QWidget* parent )
    : rviz_common::Panel( parent )
  {
    layout = new QVBoxLayout;

    //Text Box and Add Button to add new topic
    QHBoxLayout* topic_layout = new QHBoxLayout;

    add_topic_box_ = new QComboBox;
    initComboBox();
    topic_layout->addWidget( add_topic_box_ );

    QPushButton* add_topic_button_ = new QPushButton("Add Action");
    topic_layout->addWidget( add_topic_button_ );

    layout->addLayout( topic_layout );
    //End of Text Box and Add Button

    m_sigmap = new QSignalMapper(this);

    connect(m_sigmap, SIGNAL(mapped(int)),this, SLOT(OnClickDeleteButton(int)));

    //Button to send cancel topic
    QPushButton* send_topic_button_ = new QPushButton("Cancel Action");
    layout->addWidget( send_topic_button_ );

    setLayout( layout );

    connect( send_topic_button_, SIGNAL( clicked() ), this, SLOT( sendTopic ()));
    connect( add_topic_button_, SIGNAL( clicked() ), this, SLOT( addTopic() ));
  }

  void CancelAction::initComboBox(){
    add_topic_box_->addItem("");
    
    // TODO: ROS2 - Replace with proper action discovery
    // In ROS2, actions use different topic structure
    // For now, add common action names manually
    add_topic_box_->addItem("/move_base");
    add_topic_box_->addItem("/navigation");
    add_topic_box_->addItem("/manipulation");
    
    /* ROS2 topic discovery would look like:
    auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
    auto topic_names_and_types = node->get_topic_names_and_types();
    for (const auto& topic_info : topic_names_and_types) {
      // Look for action topics ending with /_action/cancel_goal
      if (topic_info.first.find("/_action/cancel_goal") != std::string::npos) {
        std::string action_name = topic_info.first;
        action_name = action_name.substr(0, action_name.find("/_action/cancel_goal"));
        add_topic_box_->addItem(action_name.c_str());
      }
    }
    */
  }


  void CancelAction::OnClickDeleteButton(int id){
    std::vector<topicListLayout>::iterator it = topic_list_layouts_.begin();
    while( it != topic_list_layouts_.end()){
      if(it->id == id){
	it->topic_name_->hide();
	delete it->topic_name_;

	it->remove_button_->hide();
	delete it->remove_button_;

	delete it->layout_;
	it->publisher_.shutdown();
	it = topic_list_layouts_.erase( it );
	Q_EMIT configChanged();
      }else{
	++it;
      }
    }
  }

  void CancelAction::addTopic()
  {
    output_topic_ = add_topic_box_->currentText();
    if( output_topic_ != "" ){
      add_topic_box_->setCurrentIndex( 0 );
      addTopicList(output_topic_.toStdString());
    }
    Q_EMIT configChanged();
  }

  void CancelAction::addTopicList(std::string topic_name){
    topicListLayout tll;

    if(!topic_list_layouts_.empty()){
      topicListLayout lastTll = topic_list_layouts_.back();
      tll.id = lastTll.id + 1;
    }else{
      tll.id = 0;
    }

    tll.layout_ = new QHBoxLayout;

    tll.topic_name_ = new QLabel( topic_name.c_str() );
    tll.layout_->addWidget( tll.topic_name_ );

    tll.remove_button_ = new QPushButton("Delete");
    tll.layout_->addWidget( tll.remove_button_ );

    layout->addLayout(tll.layout_);

    tll.publisher_ = nh_.advertise<actionlib_msgs::GoalID>( topic_name + "/cancel", 1 );
    
    topic_list_layouts_.push_back(tll);

    connect(tll.remove_button_, SIGNAL(clicked()), m_sigmap, SLOT(map()));
    m_sigmap->setMapping(tll.remove_button_, tll.id);

  }

  void CancelAction::sendTopic(){
    std::vector<topicListLayout>::iterator it = topic_list_layouts_.begin();
    while( it != topic_list_layouts_.end()){
      action_msgs::msg::GoalInfo msg;
      it->publisher_.publish(msg);
      it++;
    }
  }

  void CancelAction::save( rviz_common::Config config ) const
  {
    rviz_common::Panel::save( config );

    rviz_common::Config topic_list_config = config.mapMakeChild( "topics" );

    std::vector<topicListLayout>::const_iterator it = topic_list_layouts_.begin();
    while( it != topic_list_layouts_.end()){
      topic_list_config.listAppendNew().setValue( it->topic_name_->text() );
      it ++;
    }
    config.mapSetValue( "Topic", output_topic_ );
  }

  // Load all configuration data for this panel from the given Config object.
  void CancelAction::load( const rviz_common::Config& config )
  {
    rviz_common::Panel::load( config );
    rviz_common::Config topic_list_config = config.mapGetChild( "topics" );
    int num_topics = topic_list_config.listLength();

    for( int i = 0; i < num_topics; i++ ) {
      addTopicList(topic_list_config.listChildAt( i ).getValue().toString().toStdString());
    }
  }

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::CancelAction, rviz_common::Panel )
