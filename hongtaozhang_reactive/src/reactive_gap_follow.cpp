#include <ros/ros.h>
#include <vector>

#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <math.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

struct MAX_GAP{
    int i_start;
    int length;
};

class reactiveGap {
private:
    ros::NodeHandle n;
    ros::Publisher pub_acker;
    ros::Subscriber sub_scan;
    std::vector<float> range_laser;
    std::vector<float> angle_laser;
    std::vector<float> range_base;
    std::vector<float> angle_base;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    ackermann_msgs::AckermannDriveStamped acker;

public:
    reactiveGap(): tfListener(tfBuffer){
        pub_acker = n.advertise<ackermann_msgs::AckermannDriveStamped>("nav",1);
        sub_scan = n.subscribe("/scan",1, &reactiveGap::scan_callback, this);
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg){

        int i_nearest, fur_index;
        float fur, f_angle;
        int angle_range = 340;
        range_laser = msg->ranges;
        float min_angle = msg->angle_min;
        float angle_inc = msg->angle_increment;

        transform(min_angle, angle_inc);

        i_nearest = find_nearest(angle_range);
        float bubble = 15;
        for(int i = i_nearest -bubble; i < i_nearest + bubble; i++ )
        {
            range_base[i] = 0;
        }

        MAX_GAP max_gap =find_maxgap(angle_range );

        fur_index = (int)round((max_gap.i_start+max_gap.length*0.5));

    /*    std::cout<<"i_start:   "<<max_gap.i_start<<std::endl;
        std::cout<<"length :  "<<max_gap.length<<std::endl;
*/

        //f_angle = ((fur_index)/3)*(M_PI/180) - M_PI ;
        f_angle = angle_laser[fur_index];


        f_angle = check_steering(f_angle) ;
        acker.drive.steering_angle = f_angle;
        acker.drive.speed = determine_speed(f_angle);
        pub_acker.publish(acker);


    }

    void transform(float angle_min, float angle_increment){

        int array_size = 0;
        int real_size = 0;
        range_base.clear();
        angle_base.clear();
        angle_laser.clear();

        geometry_msgs::PointStamped lidar_pt;
        geometry_msgs::PointStamped base_pt;
        geometry_msgs::TransformStamped transformStamped;

        lidar_pt.header.frame_id = "laser";
        lidar_pt.point.z = 0;


        for (std::vector<float>::iterator it = range_laser.begin() ; it != range_laser.end(); ++it){

            array_size++;
            if(std::isnan(*it) || std::isnan(*it)){
                range_laser.erase(it);
                continue;

            }
            lidar_pt.point.x = range_laser[array_size-1]*cos(angle_min +  (array_size - 1)* angle_increment);
            //ROS_INFO("lidar_point_x: %f", lidar_pt.point.x);
            lidar_pt.point.y = range_laser[array_size-1]*sin(angle_min +  (array_size - 1)* angle_increment);
            angle_laser.push_back(angle_min +  (array_size-1)* angle_increment);
            //ROS_INFO("lidar_point_y: %f", lidar_pt.point.y);

            try {
                //bool tran = tfBuffer.canTransform("base_link","laser",ros::Time(0));
                transformStamped = tfBuffer.lookupTransform("base_link","laser", ros::Time(0));
                tf2::doTransform(lidar_pt, base_pt, transformStamped);

                //tfBuffer.transform(lidar_pt, base_pt, "base_link");//the name of the base link
               // ROS_INFO("base_point_x: %f", base_pt.point.x);
                //ROS_INFO("base_point_y: %f", base_pt.point.y);
                range_base.push_back( sqrt(pow(base_pt.point.x , 2) + pow(base_pt.point.y , 2)));
                //ROS_INFO("range_base: %f", range_base[real_size]);
                angle_base.push_back(atan(base_pt.point.y/base_pt.point.x));
                //ROS_INFO("angle_base: %f", angle_base[real_size]);
                real_size ++;
            }
            catch(tf2::TransformException &ex){
                ROS_WARN("Failure %s\n", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }


        }





    }

    int find_nearest(int angle_range){
        float nearest = 100;
        int i_nearest;

        for(int i=angle_range; i < range_base.size()-angle_range;i++) {

            if(std::isnan(range_base[i])  && std::isinf(range_base[i])  ){
                if(std::isnan(range_base[i]))
                    range_base[i] = 0;
                continue;}

            if(nearest > range_base[i]) {
                nearest = range_base[i];
                i_nearest = i;
            }
        }
        return i_nearest;
    }



    MAX_GAP find_maxgap(int angle_range){

        int count =0;
        int i_length =0;
        float filter =1.6;
        MAX_GAP max_gap = {0,0};


        for(int i=angle_range; i < range_base.size()-angle_range;i++) {
            if(i==angle_range)
            {
                if (range_base[i] >= filter){
                    count++;
                    i_length = i;}
                continue;
            }

            if (range_base[i] >=filter && range_base[i -1] < filter){
                count++;
                i_length = i;
            }

            if ((range_base[i] >=filter && range_base[i -1] >= filter) )
                count++;


            if((range_base[i] <filter && range_base[i-1] >=filter)|| (i==range_base.size()-angle_range-1))
                if(count > max_gap.length){
                    max_gap.length = count;
                    max_gap.i_start = i_length;
                    count = 0;

                } else
                    count =0;


        }
        return max_gap;

    }




    float determine_speed(float f_angle){
        if (0 <= fabs(f_angle) && fabs(f_angle) <= (10*M_PI/180)){
            return 3;
        }
        else if ((10*M_PI/180) < fabs(f_angle) && fabs(f_angle) <= (20*M_PI/180)){
            return  2;
        }
        else{
            return 2;
        }
    }

    float check_steering(float steering){

        if  (22*M_PI/180<= fabs(steering)){
            if(steering >0)
                steering = 22*M_PI/180;
            if(steering<0)
                steering = -22*M_PI/180;}
        return steering;
    }


};
int main(int argc, char** argv){
    ros::init(argc,argv,"reactive_methods");
    reactiveGap gap;
    ros::spin();
    return 0;
}