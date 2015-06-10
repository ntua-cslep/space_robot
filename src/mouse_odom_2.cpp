 #include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/input.h>
#include <fcntl.h>
#include <math.h>

#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#define L 0
#define R 1
#define F 2

class MouseOdom{
public:
  MouseOdom(std::string device, std::string frame)
  {
    this->ready = 0;
    this->dev = device;
    this->data.header.frame_id = frame;
    //openDevice();
  }

  bool openDevice()
  {
    if((this->fd = open(this->dev.c_str(), O_RDONLY | O_NONBLOCK )) == -1) 
    {
      ROS_ERROR("Device %s can't open. If exist try this command: sudo chmod a+rw %s",this->dev.c_str(),this->dev.c_str());
      exit(EXIT_FAILURE);
      return 1;
    }
    else
    {
      ROS_INFO("Device %s open OK",this->dev.c_str());
      return 0;
    }
  }

  void closeDevice()
  {
    close(this->fd);
  }

  bool readRelativeMove()
  {
    this->ready = 0;
    if(read(this->fd, &this->ie, sizeof(struct input_event))!=-1)
    {
      ROS_INFO("time %ld.%06ld\ttype %d\tcode %d\tvalue %d", ie.time.tv_sec, ie.time.tv_usec, ie.type, ie.code, ie.value);
      if (this->ie.type == EV_REL) //update x or y 
      {
        if (this->ie.code == REL_X)  this->dy = (float)(-this->ie.value);//its opposite and switched axes
        if (this->ie.code == REL_Y)  this->dx = (float)(-this->ie.value);
      }  
      else if (ie.type == EV_SYN) //update mouse position and reset for next iteration 
      {
        this->data.header.stamp.sec  = (int)this->ie.time.tv_sec;
        this->data.header.stamp.nsec = (int)this->ie.time.tv_usec*1000;
        this->data.vector.y = this->dy;
        this->data.vector.x = this->dx;
        ROS_INFO("Data ready: time %f X %f Y %f", this->data.header.stamp.toSec(), this->data.vector.x, data.vector.y);
        //reset 
        this->dy=0.0;
        this->dx=0.0;
        this->ready = 1;
      }         
    }
    return this->ready;
  }

  geometry_msgs::Vector3Stamped getVector()
  {
    if (this->ready)
    {
      this->ready = 0;
      return this->data;  
    }
    else
    {
      this->data.header.stamp = ros::Time::now();
      this->data.vector.y = 0.0;
      this->data.vector.x = 0.0;
      return this->data;  
    }
  }

private:
  int fd;
  float dx ,dy;
  bool ready;
  std::string dev;
  struct input_event ie;
  geometry_msgs::Vector3Stamped data;
};

nav_msgs::Odometry odometry(float scale, geometry_msgs::Vector3Stamped v1, geometry_msgs::Vector3Stamped v2, nav_msgs::Odometry& odom, float& th)
{
  geometry_msgs::Vector3 r1, r2;
  r1.y = 0.0825;
  r2.y = -0.0825;
  
  th += atan2((-v1.vector.x + v2.vector.x), (0.2*18867));
  odom.pose.pose.position.x += (((v1.vector.x + v2.vector.x)/2)*cos(th) + ((v1.vector.y + v2.vector.y)/2)*sin(th))/scale;
  odom.pose.pose.position.y += (((v1.vector.x + v2.vector.x)/2)*sin(th) + ((v1.vector.y + v2.vector.y)/2)*cos(th))/scale;
  ROS_INFO("odom: x %f Y %f th %f", odom.pose.pose.position.x, odom.pose.pose.position.y, th);
  geometry_msgs::Quaternion odom_quat;
  odom_quat = tf::createQuaternionMsgFromYaw(th);
  odom.pose.pose.orientation = odom_quat;
  return odom;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mouse_odom_node");
    ros::NodeHandle node;
    ros::Rate loop_rate(1000);

    //ros parameters
    std::string dev[2];
    ros::param::param<std::string>("~left_dev_event", dev[L], "/dev/input/event3"); //left mouse
    ros::param::param<std::string>("~right_dev_event", dev[R], "/dev/input/event6"); //right mouse
    std::string frame[2];
    ros::param::param<std::string>("~left_frame_id", frame[L], "left_mouse");
    ros::param::param<std::string>("~right_frame_id", frame[R], "right_mouse");
    float dpm;
    ros::param::param<float>("~dots_per_meter", dpm, 18867);
    //publishing odom
    ros::Publisher odom_pub = node.advertise<nav_msgs::Odometry>("odom", 1000);
    //transformations
    static tf2_ros::TransformBroadcaster map_broadcaster;
    geometry_msgs::TransformStamped map_trans;
    map_trans.header.frame_id = "map";
    map_trans.child_frame_id = "odom";
    map_trans.header.stamp = ros::Time::now();
    map_trans.transform.translation.x = 0;
    map_trans.transform.translation.y = 0;
    map_trans.transform.translation.z = 0;
    map_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);

    static tf2_ros::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x = 0;
    odom_trans.transform.translation.y = 0;
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);

    ros::Time start_time = ros::Time::now();
    
    MouseOdom left_mouse(dev[L], frame[L]);
    MouseOdom right_mouse(dev[R], frame[R]);
    left_mouse.openDevice();
    right_mouse.openDevice();

    geometry_msgs::Vector3Stamped data, data2;
    nav_msgs::Odometry odometer;
    odometer.pose.pose.position.x = 0.0;
    odometer.pose.pose.position.y = 0.0;
    float theta =0;

    //////////////////////////////////////////////////
    //geometry_msgs::TransformStamped transform;
    //tf2_ros::Buffer tfBuffer;
    //tf2_ros::TransformListener tfListener(tfBuffer);
    //try{
    //  transform = tfBuffer.lookupTransform("base_link", "left_mouse", ros::Time(0));
    //}
    //catch (tf2::TransformException &ex) {
    //  ROS_WARN("%s",ex.what());
    //  //continue;
    //}

    while(ros::ok())
    {

      if(left_mouse.readRelativeMove() || right_mouse.readRelativeMove())
      {
        data = left_mouse.getVector();
        data2 = right_mouse.getVector();

        odom_pub.publish(odometry(dpm, data, data2, odometer, theta));
        

        odom_trans.header.stamp = ros::Time::now();//time_stamp[0];
        odom_trans.transform.translation.x = odometer.pose.pose.position.x;
        odom_trans.transform.translation.y = odometer.pose.pose.position.x;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odometer.pose.pose.orientation;
        odom_broadcaster.sendTransform(odom_trans);
      }   	
      else
      {
        odom_trans.header.stamp = ros::Time::now();
        odom_broadcaster.sendTransform(odom_trans);

        /*drift emulation
        map_trans.header.stamp = ros::Time::now();//time_stamp[0];
        map_trans.transform.translation.x = (ros::Time::now()-start_time).toSec() *0.02;
        map_trans.transform.translation.y = (ros::Time::now()-start_time).toSec() *0.01;
        map_trans.transform.translation.z = 0;
        map_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);
        map_broadcaster.sendTransform(map_trans); */
      }



      ros::spinOnce();
      loop_rate.sleep();
    }
    left_mouse.closeDevice();
    right_mouse.closeDevice();
    return 0;
}
