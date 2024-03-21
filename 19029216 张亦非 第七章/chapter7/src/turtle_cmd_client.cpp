/**
 * Server provide a service named "print_string", message type is "std_srvs::SetBool"
 */

#include "ros/ros.h"
#include "chapter7/TurtleSrv.h"

int main(int argc, char **argv)
{
    //ros node init
    ros::init(argc, argv, "turtle_cmd_client");

    //create node handle
    ros::NodeHandle n;

    //create a client, service message type is "chapter7::TurtleSrv"
    ros::service::waitForService("/turtle_cmd");
    ros::ServiceClient turtle_cmd_client = n.serviceClient<chapter7::TurtleSrv>("/turtle_cmd");

    //create service message, message type is chapter7::TurtleSrv
    chapter7::TurtleSrv srv;
    ROS_INFO(" give commands of turtle to be controlled: ");

    ROS_INFO(" name: ");
    char name[10]; //char name[20];
    std::cin >> name;

    ROS_INFO(" state: [ move or stop ] ");
    char state[10];
    std::cin >> state;

    ROS_INFO(" linear_x: ");
    int linear_x;
    std::cin >> linear_x;

    ROS_INFO(" angular_z: ");
    int angular_z;
    std::cin >> angular_z;

    //给服务中的请求信息赋值
    srv.request.name = name;
    srv.request.state = state;
    srv.request.linear_x = linear_x;
    srv.request.angular_z = angular_z;

    ROS_INFO("Call service to command turtle [name:%s, state:%s, linear_x:%d, angular_z:%d] ",
                srv.request.name.c_str(), srv.request.state.c_str(), srv.request.linear_x, srv.request.angular_z);

        turtle_cmd_client.call(srv);

    ROS_INFO("Show turtle result : %s", srv.response.feedback.c_str());

    return 0;
}
