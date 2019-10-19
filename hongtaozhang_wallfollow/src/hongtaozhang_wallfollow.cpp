#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <math.h>


class wallFollow {
private:
float kp = 8;
float kd = 1 ;
float ki = 0;
float servo_offset = 0;
float prev_error = 0;;
float error = 0;;
float integral = 0;
float derivative = 0;
float pid_d;
ros::NodeHandle n;
ros::Publisher pub_acker;
ros::Subscriber sub_odom;
ros::Subscriber sub_scan;
std::vector<float> range;
double Dt;
double Dt1;
double Lx = 0;
double Ly = 0;
double L0x = 0;
double L0y = 0;
double deltaL;
int loop_o = 0;
int loop_l = 0;
ackermann_msgs::AckermannDriveStamped acker;
double r_left;


public:

wallFollow(){
    sub_scan = n.subscribe("/scan", 1,&wallFollow::scan_callback,this);
    sub_odom = n.subscribe("/odom", 1, &wallFollow::odom_callback, this);
    pub_acker = n.advertise<ackermann_msgs::AckermannDriveStamped>("/nav",1000);
}
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg){
    //ROS_INFO("speed: %f", msg->twist.twist.linear.x);
    if(loop_o = 0) {
        L0x = msg->pose.pose.position.x;
        L0y = msg->pose.pose.position.y;
        loop_o = 1;
    }
    else{
        Lx = msg->pose.pose.position.x;
        Ly = msg->pose.pose.position.y;
        deltaL = pow(pow(Lx-L0x,2) + pow(Ly-L0y,2),0.5);
        L0x = Lx;
        L0y = Ly;
    }

}
void scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg){
    int real_size = 0;
    int array_size =0;
    range = msg->ranges;
    int index1 = 810;
    int index2 = 630;
    double alpha;
    double theta;
    theta = M_PI/3;
    double a = range[index2];
    double b = range[index1];
    Dt = 0;
    Dt1 = 0;
    while(std::isnan(b)|| std::isinf(b)||std::isnan(a)|| std::isinf(a)){
       if(std::isnan(b)|| std::isinf(b))
        index1 = index1+1;
       if(std::isnan(a)|| std::isinf(a))
        index2 = index2+1;
    }
    alpha = atan((a*cos(theta)-b)/ (a*sin(theta)));
    Dt = b * cos(alpha);

    Dt1 = Dt + deltaL * sin(alpha);
    error =0.8 - Dt1;

    if(loop_l =0) {
        loop_l =1;
        prev_error = error;

        PID_control(range[540]);

        return;
    }

   // error = 0.6 - Dt;
    derivative = error - prev_error;
    prev_error = error;

    r_left = b;
    PID_control(range[540]);


    }

void PID_control(float forward){


    pid_d = error * kp + kd * derivative;

    double steering = - pid_d/0.1 * (4*M_PI/180);


    acker.drive.steering_angle = steering;
    if( (1 < r_left ) && (r_left< 1.8 ) && (1.9> range[270]) && (range[270] > 0.95) ){
        pid_d = (1.6 - Dt)*kp;
        steering = - pid_d/0.1 * (4*M_PI/180);

        acker.drive.steering_angle = steering;


 /*         ROS_INFO("range : 90 : %f",range[810]);
          ROS_INFO("rleft: %f ", r_left);*/


      }
    if (0 <= abs(steering)|| abs(steering) <= (10*M_PI/180)){
        acker.drive.speed = 1.5;
    }
    else if ((10*M_PI/180) < abs(steering) || abs(steering) <= (20*M_PI/180)){
        acker.drive.speed = 1.2;
    }
    else{
        acker.drive.speed = 1;
    }
    //ROS_INFO("steering: %f ", steering*180/M_PI);
      pub_acker.publish(acker);

}

};



int main(int argc, char ** argv)
{
ros::init(argc,argv,"hongtaozhang_wallfollow");
wallFollow sn;
ros::spin();
return 0;
}
