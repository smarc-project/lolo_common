#include "controller.h"
#include <chrono>
#include <stdio.h>
#include "../lolo_message_id.h"

#include "lolo_msgs/msg/topics.hpp"

using namespace std::chrono_literals;

Controller::Controller(CaptainInterFace* _lolo, rclcpp::Node* _rcl_node) {
    lolo = _lolo;
    rcl_node = _rcl_node;
};

void Controller::setup() {
    discovery_timer_ = rcl_node->create_wall_timer(500ms, std::bind(&Controller::send_discovery, this));

    //Create subscriptions
    heartbeat_sub = rcl_node->create_subscription<std_msgs::msg::Empty>(lolo_msgs::msg::Topics::HEARTBEAT_TOPIC, 1, std::bind(&Controller::heartbeat_callback, this, std::placeholders::_1));
    abort_sub = rcl_node->create_subscription<std_msgs::msg::Empty>(lolo_msgs::msg::Topics::LOLO_ABORT_TOPIC, 1, std::bind(&Controller::abort_callback, this, std::placeholders::_1));
    
    rudder_cmd_sub = rcl_node->create_subscription<std_msgs::msg::Float32>(lolo_msgs::msg::Topics::RUDDER_CMD, 1, std::bind(&Controller::rudder_callback, this, std::placeholders::_1));
    elevator_cmd_sub = rcl_node->create_subscription<std_msgs::msg::Float32>(lolo_msgs::msg::Topics::ELEVATOR_CMD, 1, std::bind(&Controller::elevator_callback, this, std::placeholders::_1));
    elevon_port_cmd_sub = rcl_node->create_subscription<std_msgs::msg::Float32>(lolo_msgs::msg::Topics::ELEVON_PORT_CMD, 1, std::bind(&Controller::elevon_port_callback, this, std::placeholders::_1));
    elevon_strb_cmd_sub = rcl_node->create_subscription<std_msgs::msg::Float32>(lolo_msgs::msg::Topics::ELEVON_STRB_CMD, 1, std::bind(&Controller::elevon_strb_callback, this, std::placeholders::_1));
    
    thruster_port_cmd_sub = rcl_node->create_subscription<std_msgs::msg::Float32>(lolo_msgs::msg::Topics::THRUSTER_PORT_CMD, 1, std::bind(&Controller::thruster_port_callback, this, std::placeholders::_1));
    thruster_strb_cmd_sub = rcl_node->create_subscription<std_msgs::msg::Float32>(lolo_msgs::msg::Topics::THRUSTER_STRB_CMD, 1, std::bind(&Controller::thruster_strb_callback, this, std::placeholders::_1));
    vertical_thruster_1_cmd_sub = rcl_node->create_subscription<std_msgs::msg::Float32>(lolo_msgs::msg::Topics::VERTICAL_THRUSTER_1_CMD, 1, std::bind(&Controller::vertical_thruster_1_callback, this, std::placeholders::_1));
    vertical_thruster_2_cmd_sub = rcl_node->create_subscription<std_msgs::msg::Float32>(lolo_msgs::msg::Topics::VERTICAL_THRUSTER_2_CMD, 1, std::bind(&Controller::vertical_thruster_2_callback, this, std::placeholders::_1));
    vertical_thruster_3_cmd_sub = rcl_node->create_subscription<std_msgs::msg::Float32>(lolo_msgs::msg::Topics::VERTICAL_THRUSTER_3_CMD, 1, std::bind(&Controller::vertical_thruster_3_callback, this, std::placeholders::_1));
    vertical_thruster_4_cmd_sub = rcl_node->create_subscription<std_msgs::msg::Float32>(lolo_msgs::msg::Topics::VERTICAL_THRUSTER_4_CMD, 1, std::bind(&Controller::vertical_thruster_4_callback, this, std::placeholders::_1));
    
    menu_sub = rcl_node->create_subscription<std_msgs::msg::String>("topic", 1, std::bind(&Controller::menu_callback, this, std::placeholders::_1));
    
    usbl_sub = rcl_node->create_subscription<std_msgs::msg::String>("topic", 1, std::bind(&Controller::usbl_callback, this, std::placeholders::_1));
    
    satelite_sub = rcl_node->create_subscription<std_msgs::msg::String>("topic", 1, std::bind(&Controller::satelite_callback, this, std::placeholders::_1));
}

void Controller::send_discovery() {
    printf("Sending discovery package\n");
    //Send something to lolo so it can get the ip of the interface computer
    lolo->new_package(0);
    lolo->send_package();
}

//ROS2 callback functions

void Controller::abort_callback(const std_msgs::msg::Empty::SharedPtr _msg) {
    lolo->new_package(SC_ABORT); // Tell captain to go into emergency mode
    lolo->send_package();
}

void Controller::heartbeat_callback(const std_msgs::msg::Empty::SharedPtr _msg) {
    lolo->new_package(SC_HEARTBEAT); // Heartbeat message
    lolo->send_package();
}

void Controller::drop_weight_callback(const std_msgs::msg::Empty::SharedPtr _msg) {
    lolo->new_package(SC_DROP_WEIGHT); // Tell captain to go into emergency mode
    lolo->add_byte(1);
    lolo->send_package();
}

void Controller::rudder_callback(const std_msgs::msg::Float32::SharedPtr _msg) {
    float angle = _msg->data;
    lolo->new_package(SC_SET_RUDDER);
    lolo->add_float(angle);
    lolo->send_package();
}

void Controller::elevator_callback(const std_msgs::msg::Float32::SharedPtr _msg) {
    float angle = _msg->data;
    lolo->new_package(SC_SET_ELEVATOR);
    lolo->add_float(angle);
    lolo->send_package();
}

void Controller::elevon_port_callback(const std_msgs::msg::Float32::SharedPtr _msg) {
    float angle = _msg->data;
    lolo->new_package(SC_SET_ELEVON_PORT);
    lolo->add_float(angle);
    lolo->send_package();
}

void Controller::elevon_strb_callback(const std_msgs::msg::Float32::SharedPtr _msg) {
    float angle = _msg->data;
    lolo->new_package(SC_SET_ELEVON_STRB);
    lolo->add_float(angle);
    lolo->send_package();
}

void Controller::thruster_port_callback(const std_msgs::msg::Float32::SharedPtr _msg) {
    lolo->new_package(SC_SET_THRUSTER_PORT);
    lolo->add_float(_msg->data);
    lolo->send_package();
}

void Controller::thruster_strb_callback(const std_msgs::msg::Float32::SharedPtr _msg) {
    lolo->new_package(SC_SET_THRUSTER_STRB);
    lolo->add_float(_msg->data);
    lolo->send_package();
}

void Controller::vertical_thruster_1_callback(const std_msgs::msg::Float32::SharedPtr _msg) {
    lolo->new_package(SC_SET_VTHRUSTER_1);
    lolo->add_float(_msg->data);
    lolo->send_package();
}

void Controller::vertical_thruster_2_callback(const std_msgs::msg::Float32::SharedPtr _msg) {
    lolo->new_package(SC_SET_VTHRUSTER_2);
    lolo->add_float(_msg->data);
    lolo->send_package();
}

void Controller::vertical_thruster_3_callback(const std_msgs::msg::Float32::SharedPtr _msg) {
    lolo->new_package(SC_SET_VTHRUSTER_3);
    lolo->add_float(_msg->data);
    lolo->send_package();
}

void Controller::vertical_thruster_4_callback(const std_msgs::msg::Float32::SharedPtr _msg) {
    lolo->new_package(SC_SET_VTHRUSTER_4);
    lolo->add_float(_msg->data);
    lolo->send_package();
}

void Controller::menu_callback(const std_msgs::msg::String::SharedPtr _msg) {
    lolo->new_package(SC_MENUSTREAM);
    std::string s = _msg->data;
    uint8_t bytes = std::min(200, (int) s.size());
    lolo->add_byte(bytes);
    for(int i=0;i<bytes;i++) {
      lolo->add_byte(s[i]);
    }
    lolo->send_package();
}

void Controller::usbl_callback(const std_msgs::msg::String::SharedPtr _msg) {
    lolo->new_package(SC_USBL_TRANSMIT);
    std::string s = _msg->data;
    uint8_t bytes = std::min(120, (int) s.size());
    lolo->add_byte(bytes);
    for(int i=0;i<bytes;i++) {
      lolo->add_byte(s[i]);
    }
    lolo->send_package();
}

void Controller::satelite_callback(const std_msgs::msg::String::SharedPtr _msg) {
    lolo->new_package(SC_SATELITE_TRANSMIT);
    std::string s = _msg->data;
    uint8_t bytes = std::min(120, (int) s.size());
    lolo->add_byte(bytes);
    for(int i=0;i<bytes;i++) {
      lolo->add_byte(s[i]);
    }
    lolo->send_package();
}
