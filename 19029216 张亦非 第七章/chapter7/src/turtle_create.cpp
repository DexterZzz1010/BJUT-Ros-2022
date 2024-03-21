#include "ros/ros.h"
#include "turtlesim/Spawn.h"
#include <iostream>

int main(int argc, char **argv)
{
    //ros node init
    ros::init(argc, argv, "turtle_create");

    //create node handle
    ros::NodeHandle n;

    //create a client, service message type is "turtlesim::Spawn"
    ros::service::waitForService("/spawn");
    ros::ServiceClient client = n.serviceClient<turtlesim::Spawn>("/spawn");

    //create service message, message type is turtlesim::Spawn
    turtlesim::Spawn srv;

    int frontier = 11 * 10; 
    float pi = 3.1415926;
    srand((unsigned)time(NULL));
    float pose_x = (rand() % frontier) / 10.0; //海龟x和y边界是11(m)，随机生成坐标，精确到小数点后一位
    float pose_y = (rand() % frontier) / 10.0;
    float pose_theta = (rand() % 360) * pi / 180; //生成(0-359°)角度，然后将其转换成弧度
    ROS_INFO(" random pose calculating done ! ");

    ROS_INFO(" name of new-born turtle is: [name should be within 20 chars]");
    char turtle_name[20];
    std::cin >> turtle_name;

    //将上述定义海龟的参数赋给服务请求
    srv.request.name  = turtle_name;
    srv.request.x = pose_x;
    srv.request.y = pose_y;
    srv.request.theta = pose_theta;

    // client pub request to service, wait service response 
    if(client.call(srv))//"call" is a blocking function, 
    {
        ROS_INFO("Create a new turtle: %s", srv.response.name.c_str());
        // ROS_INFO("New turtle pose is : x:%.4f, y:%.4f, theta:%.4f", srv.response.x, srv.response.y, srv.response.theta);
    }
    else
    {
        ROS_ERROR("Failed to call service print_string");
        return 1;
    }

    return 0;
}
