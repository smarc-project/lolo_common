

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
//using std::placeholders::_1;

#include <iostream>
#include <boost/asio.hpp>

#include <udpinterface/UDPInterface.h>
#include "view/view.h"
#include "controller/controller.h"

using namespace boost::asio;
using ip::udp;
using std::string;
using std::cout;
using std::endl;
UDPInterface captain;

class LoLoInterface : public rclcpp::Node
{
private:

    //UDP socket endpoint and io serice
    boost::asio::io_service io_service;
    udp::endpoint receiver_endpoint;
    udp::socket* udp_socket;

    UDPInterface lolo_hw_interface;

    //rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    //LoloInterface_model model = LoloInterface_model 
    View view = View(&lolo_hw_interface, this);
    Controller control = Controller(&lolo_hw_interface, this);

public:
    LoLoInterface() : Node("lolo_interface")
    {
        rclcpp::Parameter lolo_ip_param;
        rclcpp::Parameter lolo_udp_port_param;
        this->declare_parameter("lolo_ip_str", "192.168.1.90");
        this->declare_parameter("lolo_udp_port_param", 8888);
        this->get_parameter("lolo_ip_str", lolo_ip_param);
        this->get_parameter("lolo_udp_port_param", lolo_udp_port_param);
        
        std::string ip_str = lolo_ip_param.as_string();
        ip::address lolo_ip = ip::address::from_string(ip_str);
        int port_int = lolo_udp_port_param.as_int();

        receiver_endpoint.address(lolo_ip);
        receiver_endpoint.port(port_int);
        udp_socket = new udp::socket(io_service, udp::endpoint(udp::v4(), 8888));

        //Setup publishers
        view.setup();

        //Set callback for data coming from lolo
        auto fp = std::bind(&View::data_callback, view);
        lolo_hw_interface.setCallback(fp);

        //Open connection to lolo
        lolo_hw_interface.setup(udp_socket, &receiver_endpoint);

        //setup subscribers
        control.setup();
    }

    ~LoLoInterface()
    {
        udp_socket->close();
        delete(udp_socket);
    }

private:

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr lolo_interface = std::make_shared<LoLoInterface>();
    rclcpp::spin(lolo_interface);
    rclcpp::shutdown();
    return 0;
}
