#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <iostream>
// TODO: include ROS msg type headers and libraries

class Safety {
// The class that handles emergency braking
private:
    ros::NodeHandle n;
    double speed;
    ros::Publisher pub_acker;
    ros::Publisher pub_bool;
    ros::Subscriber sub_scan;
    ros::Subscriber sub_odom;
    std::vector<float> range;
    std::vector<float> angle;
    std::vector<float> TTC_array;
    float TTC;
    ackermann_msgs::AckermannDriveStamped acker;
    std_msgs::Bool brake_bool;


    // TODO: create ROS subscribers and publishers

public:
    Safety() {
        //n = ros::NodeHandle();
        speed = 0.0;
        sub_scan = n.subscribe("/scan", 1, &Safety::scan_callback, this);
        sub_odom = n.subscribe("/odom", 1, &Safety::odom_callback, this);
        pub_acker = n.advertise<ackermann_msgs::AckermannDriveStamped>("/brake",1000);
        pub_bool = n.advertise<std_msgs::Bool>("/brake_bool",1000);

        /*
        One publisher should publish to the /brake topic with an
        ackermann_msgs/AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a
        std_msgs/Bool message.

        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        
    }
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
        // TODO: update current speed

        speed = odom_msg->twist.twist.linear.x;
        //ROS_INFO("speed %f", speed);

    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
        // TODO: calculate TTC
        int real_size = 0;
        int array_size =0;
        angle.clear();
        TTC_array.clear();
        range = scan_msg ->ranges;
        float min_range = scan_msg ->angle_min ;

       for (std::vector<float>::iterator it = range.begin() ; it != range.end(); ++it){
           array_size++;
          if(std::isnan(*it) || std::isnan(*it)){
              range.erase(it);
              continue;

          }

           angle.push_back(min_range +  (array_size-1)* scan_msg->angle_increment);
           real_size ++;
       }

       for (int i = 0; i<real_size; i++){
           TTC_array.push_back( range[i]/std::max(0.00, speed*cos(angle[i])));

       }

       std::sort(TTC_array.begin(), TTC_array.end());
       TTC = TTC_array[0];
       //ROS_INFO("TTC %f", TTC);
       if (TTC < 0.4)
       {
           acker.drive.speed = 0.0;
           brake_bool.data = true;

           pub_bool.publish(brake_bool);
           pub_acker.publish(acker);
       }
        brake_bool.data = false;
        pub_bool.publish(brake_bool);
        // TODO: publish drive/brake message
    }

};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "hongtaozhang_safety");
    Safety sn;
    ros::spin();
    return 0;
}
