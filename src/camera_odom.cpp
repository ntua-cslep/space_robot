/*
    Simple ROS enabled udp server
    publishing  data 2D pose type
    received from CameraServo.exe
    that localize  space robot by
    using MatrixVision Camera
*/
#include <stdio.h> //printf
#include <string.h> //memset
#include <stdlib.h> //exit(0);

#include <arpa/inet.h>
#include <sys/socket.h>
#include <stdint.h>
#include <inttypes.h>

#include <sstream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>


#define BUFLEN 512  //Max length of buffer
#define DEFAULT_PORT 25000  //The default port on which to listen for incoming data
#define ROBOT_ID 11 //this is the default identification number used also in camera program 

void die(char *s)
{
    perror(s);
    exit(1);
}
 
int main(int argc, char **argv)
{
    /*
     * Ros initialize
     */  
    ros::init(argc, argv, "udp_server");
    ros::NodeHandle nh;
    ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("camera/odom", 1000);
    ros::Rate loop_rate(500);
    nav_msgs::Odometry odom;
    short x,y,th,u,v,w,t;
    int rob_id;
    ros::param::param<int>("~robot_id", rob_id, ROBOT_ID);
    ROS_INFO("Reading odometry for Robot with ID: %d\n", rob_id);
    //map to odom transformation
    static tf2_ros::TransformBroadcaster map_broadcaster;
    geometry_msgs::TransformStamped map_trans;
    map_trans.header.frame_id = "odom";
    map_trans.child_frame_id = "map";
    /*
     * UDP setup
     */
    int port;
    ros::param::param<int>("~udp_port", port, DEFAULT_PORT);
    ROS_INFO("listening to port: %d\n", port);

    struct sockaddr_in si_me, si_other;
     
    int s, i;
    uint slen = sizeof(si_other);
    ssize_t recv_len=0;
    char buf[BUFLEN]="";
     
    //create a UDP socket
    if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) // IP addressing, UDP protocol, 
    {
        ROS_ERROR("Can't create UDP socket");
    }
     
    // zero out the structure
    memset((char *) &si_me, 0, sizeof(si_me));
     
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(port);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);
     
    //bind socket to port
    if( bind(s , (struct sockaddr*)&si_me, sizeof(si_me) ) == -1)
    {
        ROS_ERROR("Can't bind UDP socket");
    }
     
    //keep listening for data
    while(ros::ok())
    {                    
        //try to receive some data, this is a blocking call
        if ((recv_len = recvfrom(s, &buf, BUFLEN, 0, (struct sockaddr *)&si_other, &slen)) == -1)
            ROS_WARN("Can't receive from UDP socket");
        
        //message parser
        if (buf[0]==0x7E & buf[16]==0x7E)//delimiters
        {
            if (buf[1] == rob_id)
            {   
                memcpy(&x,  &buf[2], 2);// X in 10^-4 meters
                memcpy(&y,  &buf[4], 2);// Y
                memcpy(&th, &buf[6], 2);// theta 10^-4 rad 
                memcpy(&u,  &buf[8], 2);// Xdot in 10^-4 meters/sec
                memcpy(&v, &buf[10], 2);// Ydot
                memcpy(&w, &buf[12], 2);// thetaDot 10^-4 rad/sec
                memcpy(&t, &buf[14], 2);// latency ms

            //Odometry broadcast
                //header
                odom.header.stamp = ros::Time::now();//as soon as possible minus the latency
                odom.header.frame_id = "odom";//reference franme for position
                odom.child_frame_id  = "map";//reference franme for velocity         
                //set position
                odom.pose.pose.position.x = (double)(x *0.0001);
                odom.pose.pose.position.y = (double)(y *0.0001);
                odom.pose.pose.position.z = 0.43;                
                //set orientation
                geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw((double)(th *0.0001));
                odom.pose.pose.orientation= odom_quat;
                //set velocitiy
                odom.twist.twist.linear.x = (double)(u *0.0001);
                odom.twist.twist.linear.y = (double)(v *0.0001);
                odom.twist.twist.angular.z= (double)(w *0.0001); 

            //transformation broadcast
                ros::Duration delay((float)(t*0.001));
                map_trans.header.stamp = ros::Time::now() - delay;
                map_trans.transform.translation.x = (double)(x *0.0001);;
                map_trans.transform.translation.y = (double)(y *0.0001);;
                map_trans.transform.translation.z = 0.0;
                map_trans.transform.rotation = tf::createQuaternionMsgFromYaw((double)(th *0.0001));
                map_broadcaster.sendTransform(map_trans);           
            }
            else ROS_INFO("different Robot id recieved. ID: %d", (int)buf[1]);
        }

        //
        pub_odom.publish(odom);

        //print details of the client/peer and the data received
        ROS_INFO("Received packet from %s:%d\n", inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
        ROS_INFO("Receivied data: Rob_ID: %d x %d, y %d, th %d, u %d, v %d, w %d, t %d\n" , buf[1], x, y, th, u, v, w, t);

        ros::spinOnce();
        loop_rate.sleep();//consider removing
    }
 
    close(s);
    return 0;
}