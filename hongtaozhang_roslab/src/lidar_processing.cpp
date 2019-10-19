#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include "hongtaozhang_roslab/scan_range.h"


class PubandSub
{
private:
    ros::NodeHandle n;
    ros::Publisher pub_closest;
    ros::Publisher pub_farthest;
    ros::Publisher pub_scan;
    ros::Subscriber sub;
    std_msgs::Float64 closest;
    std_msgs::Float64 farthest;
    hongtaozhang_roslab::scan_range scan_data;
    int real_size = 0;
    std::vector<float> range;

    void BubbleSort(std::vector<float> & range, int realSize)
    {

        for(int m = 1; m < realSize; m++)
        {
            for(int i = 0; i< realSize-m; i++)
            {
                if(range[i] > range[i+1])
                {
                    float temp = range[i+1];
                    range[i+1] = range[i];
                    range[i] = temp;
                }
            }
        }

    }
    int Filter(std::vector<float> &range, int size){
        int real = 0;
        for (int i = 0; i < size; i++) {
            if( std::isnan(range[i]) || std::isinf(range[i]))
                continue;
            range[real] = range[i];
            real++;
        }

        return real;
    }

public:
    PubandSub(){
        pub_closest = n.advertise<std_msgs::Float64>("/closest_point", 1000);
        pub_farthest = n.advertise<std_msgs::Float64>("/farthest_point", 1000);
        pub_scan = n.advertise<hongtaozhang_roslab::scan_range>("/scan_range", 1000);
        sub = n.subscribe("/scan", 1, &PubandSub::lidarCallback,this);
    }
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {


        const int array_size = msg->ranges.size();
        range = msg->ranges;
        real_size = Filter( range, array_size);
        BubbleSort(range, real_size);
        closest.data = range[0];
        farthest.data = range[real_size - 1];

        scan_data.maximum_range = farthest.data;
        scan_data.minimum_range = closest.data;

        pub_closest.publish(closest);
       //ROS_INFO("The closest point distance is %f\n", closest.data);
        pub_farthest.publish(farthest);
        //ROS_INFO("The farthest point distance is %f\n", farthest.data);
        pub_scan.publish(scan_data);
        //ROS_INFO("minimum %f\n", scan_data.minimum_range);
        //ROS_INFO("maximum %f\n", scan_data.maximum_range);


    }

};



int main(int argc, char** argv){
ros::init(argc, argv, "lidar_processing");

  PubandSub A;
  ros::spin();


return 0;
}
