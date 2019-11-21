#include <ros/ros.h>
#include <math.h>
#include <fstream>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <string.h>
struct ODOM{
    double x;
    double y;
    double yaw;
};
class pure_pursuit{
private:
    ros::NodeHandle n;
    ros::Subscriber pf_sub;
    ros::Publisher acker_pub;
    ros::Publisher vis_pub;
    std::vector<float> waypointX;
    std::vector<float> waypointY;
    double L = 1.15;
    double L_update =1.15;
    double r;
    double gamma;
    double steering, velocity;
    double Kp = 0.25;
//    double roll, pitch, yaw;
//    double x, y;
//    double delta_x, delta_y;
//    double dis, gap, min_gap;
//    int k = 0;
//    double x_d, y_d, L_d_square;

public:
    pure_pursuit() {
        // read the waypoints file
        std::ifstream file("/home/hunter/hongtaozhang_ws/src/ESE-680-Autonomous-Racing/pure_pursuit/waypoints.csv");
        std::string s;
        int i;

        vis_pub = n.advertise<visualization_msgs::Marker>("waypoints_marker", 1);

        //read x and y ,devided by ',', from each line
        while (file.good()) {
            getline(file, s, ',');
            waypointX.push_back(stod(s)); // convert string to double
            getline(file, s, '\n');
            waypointY.push_back(stod(s));

        }
        file.close();


        visualization_msgs::Marker marker;

        //configure the markers
        marker.header.frame_id = "map";
        marker.ns = "waypoints_vis";
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;

        // two rounds to enable the makers circling the track
        for (auto j=0; j<2; ++j){
        for (i = 0; i < waypointY.size(); i++) {

            marker.id = i;
            marker.header.stamp = ros::Time::now();
            marker.pose.position.x = waypointX[i];
            marker.pose.position.y = waypointY[i];

            vis_pub.publish(marker);
            ros::Duration(0.01).sleep();
        }
        }

        pf_sub = n.subscribe("pf/pose/odom", 100, &pure_pursuit::pure_pursuit_callback, this);
        acker_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("nav", 1);

    }


    void pure_pursuit_callback(const nav_msgs::Odometry::ConstPtr &msg){


        ODOM odom;
        double roll,pitch,yaw;
        double delta_x, delta_y;
        int k = 0;
        std::vector<float> x_car(waypointX.size(),0);
        std::vector<float> y_car(waypointY.size(),0);
        double closest = DBL_MAX;

        //get the car's position in the global frame from particle filter.
        odom.x = msg->pose.pose.position.x;
        odom.y = msg->pose.pose.position.y;


        //get the RPY from rotation
        tf::Quaternion qua;
        tf::quaternionMsgToTF(msg->pose.pose.orientation,qua);
        tf::Matrix3x3(qua).getRPY(roll,pitch, yaw);
        odom.yaw = yaw;
        ROS_INFO("yaw:%f", yaw);

        // calculate the waypoints in car's frame
        for(int i = 0; i<waypointY.size(); i++) {

            double delta_x = waypointX[i] - odom.x;
            double delta_y = waypointY[i] - odom.y;

            x_car[i] = delta_x * cos(odom.yaw) + delta_y * sin(odom.yaw);
            y_car[i] = -delta_x * sin(odom.yaw) + delta_y * cos(odom.yaw);

        }

        //calculate the distances between the car and all waypoints
        // and find the closest one relative to the look ahead distance.
        for(int i = 0; i<waypointY.size(); i++){

            double dist = pow(pow(x_car[i],2) + pow(y_car[i],2),0.5);
            double gap = fabs(L - dist);

            // the closest is set to inf initially
            if (closest > gap && x_car[i] >= 0.0) {

                closest = gap;
                k = i;
                L_update = dist;

            }
        }

        // calculate the gamma and radius for the turning--core of the pure pursuit
        double x_d = x_car[k];
        double y_d = y_car[k];
        double Ls = x_d*x_d + y_d*y_d;

        r = Ls/(2*(y_car[k]));
        gamma = 1/r;

        std::cout<<k<<std::endl;

        // Set the steering angle according to the curvature
        //Attention: abs() is used for the integer and for float, fabs() is the correct function.
        steering = Kp*gamma;
        steering = steering>0 ? std::min(steering, 0.4189) : std::max(steering, -0.4189);
        velocity = fabs(steering*180/M_PI)<=10 ? 5 : 2 ;

        //L = L_update;
        ackermann_msgs::AckermannDriveStamped acker;

        //Publish the driving message
        acker.header.stamp = ros::Time::now();
        acker.header.frame_id = "laser";
        acker.drive.speed = velocity;
        acker.drive.steering_angle = steering;
        acker_pub.publish(acker);

//        x = (msg->pose).pose.position.x;
//        y = (msg->pose).pose.position.y;
//
//        tf::Quaternion q(
//                (msg->pose).pose.orientation.x,
//                (msg->pose).pose.orientation.y,
//                (msg->pose).pose.orientation.z,
//                (msg->pose).pose.orientation.w);
//
//        tf::Matrix3x3 m(q);
//        m.getRPY(roll, pitch, yaw);
//
//        std::vector<double> x_car(waypointX.size(), 0);
//        std::vector<double> y_car(waypointY.size(), 0);
//
//        for (auto i=0; i<waypointX.size(); ++i){
//
//            delta_x = waypointX[i] - x;
//            delta_y = waypointY[i] - y;
//
//            y_car[i] = -delta_x*sin(yaw) + delta_y*cos(yaw);
//            x_car[i] = delta_x*cos(yaw) + delta_y*sin(yaw);
//
//        }
//
//        min_gap = DBL_MAX;
//
//        for (auto i=0; i<x_car.size(); ++i){
//
//            dis = sqrt(x_car[i]*x_car[i] + y_car[i]*y_car[i]);
//            gap = std::abs(dis-L);
//
//            if ((gap < min_gap) && (x_car[i]>=0.0)){
//
//                k = i;
//                min_gap = gap;
//
//            }
//
//        }
//        std::cout << k << std::endl;
//        x_d = x_car[k];
//        y_d = y_car[k];
//        L_d_square = x_d*x_d + y_d*y_d;
//        gamma = 2*y_d/L_d_square;
//        steering = Kp*gamma;
//        steering = std::min(steering, 0.4189);
//        steering = std::max(steering, -0.4189);
//
//        if (abs(steering*180/M_PI)<10){
//            velocity = 5;
//        }else if (abs(steering*180/M_PI)<20){
//            velocity = 2;
//        }else{
//            velocity = 2;
//        }
//
//        ackermann_msgs::AckermannDriveStamped acker;
//        acker.header.stamp = ros::Time::now();
//        acker.header.frame_id = "laser";
//        acker.drive.steering_angle = steering;
//        acker.drive.speed = velocity;
//        acker_pub.publish(acker);

    }
};
// main function
int main(int argc, char **argv){
    ros::init(argc, argv,"pure_pursuit");
    pure_pursuit purePursuit;
    ros::spin();
    return 0;
}
