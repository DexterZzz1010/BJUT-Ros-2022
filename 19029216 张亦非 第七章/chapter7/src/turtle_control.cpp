#include <ros/ros.h>
#include <turtlesim/Spawn.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

ros::Publisher turtle_vel;

void poseCallback(const turtlesim::PoseConstPtr& msg)
{
    ROS_INFO("Turtle2 position:(%f %f)",msg->x,msg->y);
}

int main(int argc,char** argv)
{
    //初始化节点
    ros::init(argc,argv,"turtle_control");
    
    ros::NodeHandle node;

    //通过服务调用，产生第二只乌龟turtle2
    ros::service::waitForService("spawn");
    ros::ServiceClient add_turtle=node.serviceClient<turtlesim::Spawn>("spawn");
    turtlesim::Spawn srv;
    srv.request.x=5;
    srv.request.y=5;
    srv.request.name="turtle2";
    add_turtle.call(srv);
    
    ROS_INFO("The turtle2 has been spawn!");

    //订阅乌龟的pose信息
    ros::Subscriber sub=node.subscribe("turtle2/pose",10,&poseCallback);

    //定义turtle的速度控制发布器
    turtle_vel=node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel",10);

    ros::Rate rate(10.0);
    
    while(node.ok())
    {
        //发布速度控制命令
        geometry_msgs::Twist vel_msg;
        vel_msg.angular.z=1;
        vel_msg.linear.x=1;
        turtle_vel.publish(vel_msg);
    
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
