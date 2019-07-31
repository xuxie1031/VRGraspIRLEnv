#pragma once

#include <ros/ros.h>

#include <boost/thread.hpp>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <fcntl.h>

#include <deque>
#include <vector>
#include <string>
#include <iostream>
#include <unistd.h>

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#include <std_msgs/String.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>

#include "huroco_right_arm/right_arm_server.h"

using namespace rapidjson;

#define TCP_BUFFER_SIZE 100000

class DataReceiver{

public:
    DataReceiver(int port, std::string tag);
    ~DataReceiver();

private:
    unsigned int MessagePublishDemo(const unsigned char* Data);
    unsigned int MessagePublishTrain(const unsigned char* Data);
    void BufferedRecv();

    ros::NodeHandle nh;
    ros::Publisher tf_pub;
    ros::Publisher str_pub;
    ros::ServiceClient init_client;
    ros::ServiceClient cartesian_client;
    std::string pub_tag;

    int socket_desc;
    int client_sock;
    struct sockaddr_in server, client;
    bool ready2connect;

    unsigned char* DataReceived;
    unsigned int BytesReceived;
    unsigned int BufferSize;

    float sleeping_time;
};

class DataSender{

public:
    DataSender(int port, std::string ip_addr, std::string tag);
    ~DataSender();

private:
    void ProcDataRecv();
    void OnDataRecvDemo(const tf2_msgs::TFMessage::ConstPtr& msg);
    void OnDataRecvTrain(const std_msgs::String::ConstPtr& msg);

    ros::NodeHandle nh;
    ros::Subscriber tf_sub;
    ros::Subscriber str_sub;
    std::string sub_tag;

    bool sock_connected;
    int sock;
    struct sockaddr_in server;
    int cnt;
};