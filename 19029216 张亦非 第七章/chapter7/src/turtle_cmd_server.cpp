/**
 * Server provide a service named "print_string", message type is "std_srvs::SetBool"
 */

#include "ros/ros.h"
#include "chapter7/TurtleSrv.h"
#include "geometry_msgs/Twist.h"

bool turtleCallback(chapter7::TurtleSrv::Request &req, 
                    chapter7::TurtleSrv::Response &res)
{
    //state=move时，指定的海龟根据指定参数运动
    if (req.state == "move")
    {

        ros::NodeHandle n1;
        ros::Publisher pub = n1.advertise<geometry_msgs::Twist>("/"+req.name + "/cmd_vel", 10);
        ros::Rate loop_rate(1);
        
        //按照循环频率发布海龟运动信息，会被turtlesim接收并控制海龟运动
        while (ros::ok()) {
            geometry_msgs::Twist msg;
            msg.linear.x = req.linear_x;
            msg.angular.z = req.angular_z;
            ROS_INFO("vel_publish: linear_x=%f angular_z=%f", msg.linear.x, msg.angular.z);
            pub.publish(msg);
            loop_rate.sleep();
            ros::spinOnce();
        }
		//输出反馈信息
        res.feedback = req.name + " has been controlled ! ";
        return true;
    }

    //state=stop时，指定的海龟停止
    else if (req.state == "stop")
    {
        ros::NodeHandle n1;
        ros::Publisher pub = n1.advertise<geometry_msgs::Twist>("/"+req.name + "/cmd_vel", 10);
        ros::Rate loop_rate(1);
        
        //按照循环频率发布海龟运动信息，会被turtlesim接收并控制海龟运动
        while (ros::ok()) {
            geometry_msgs::Twist msg;
            msg.linear.x = 0;
            msg.angular.z = 0;
            ROS_INFO("vel_publish: linear_x=%f angular_z=%f", msg.linear.x, msg.angular.z);
            pub.publish(msg);
            loop_rate.sleep();
            ros::spinOnce();
        }
        //输出反馈信息
        res.feedback = req.name + " has been controlled ! ";
        return true;
    }

    else
    {
        //输出反馈信息
        res.feedback = " wrong command ! ";
        return false;
    }
}

int main(int argc, char **argv)
{
    //ros node init
    ros::init(argc, argv, "turtle_cmd_server");

    //create node handle
    ros::NodeHandle n;

    //create a Server named "turtle_cmd", regisite callback function "turtleCallback"
    ros::ServiceServer turtle_cmd_service = n.advertiseService("/turtle_cmd", turtleCallback);

    //loop wait callback
    ROS_INFO("Ready to print show turtle information.");
    ros::spin();

    return 0;
}
