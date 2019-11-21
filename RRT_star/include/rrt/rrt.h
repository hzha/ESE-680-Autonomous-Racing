// ESE 680

// This file contains the class definition of tree nodes and RRT

// ros
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

// standard
#include <math.h>
#include <vector>
#include <array>
#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <random>

// Struct defining the Node object in the RRT tree.
// More fields could be added to thiis struct if more info needed.
// You can choose to use this or not
typedef struct Node {
    double x, y;
    double cost; // only used for RRT*
    int parent; // index of parent node in the tree vector
    bool is_root = false;
} Node;


class RRT {
public:
    RRT(ros::NodeHandle &nh);
    virtual ~RRT();


private:
    ros::NodeHandle nh_;
    std::vector<float> range_laser;

    int inflation_size = 4;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    nav_msgs::OccupancyGrid occumap;
    boost::shared_ptr<nav_msgs::OccupancyGrid const> levine_map_ptr;
    visualization_msgs::Marker tree_line_marker;
    visualization_msgs::Marker path_line_marker;

    float step_size = 0.2;
    double near_range = 0.6;
    float goal_threshold = 0.1;
    int MAX_ITER = 100;
    int interpolation_number = 10;
    int sample_goal_prob_reciprocal=5;

    //waypoint stuff
    std::vector<double> waypoint_x_data;//the x-coordinate of waypoints
    std::vector<double> waypoint_y_data;//the y-coordinate of waypoints
    std::string temp;

    //self set to view the waypoints
    double roll, pitch, yaw;
    double dis, gap, min_gap_path, min_gap_wp;
    int k = 0, n = 0;
    double lookahead_wp = 1.46;
    double lookahead_path = 0.48;
    double Kp = 0.2;   //proportional gain between curvature and steering angle
    double x_d, y_d, L_d_square, gamma;
    double angle;
    double velocity;
    std::vector<Node> path_old;

    int map_erase_count = 0;
    int map_clear_pubnumber =100;
    ackermann_msgs::AckermannDriveStamped drive_msg;

    // ros pub/sub

    ros::Publisher map_pub_;
    ros::Publisher vis_pub;
    ros::Publisher drive_pub;
    ros::Publisher tree_line_pub;
    ros::Publisher path_line_pub;
    ros::Subscriber pf_sub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber meta_sub_;


    // tf stuff
    tf::TransformListener listener;


    // random generator, use this
    std::random_device rd;
    std::mt19937 gen{rd()};


    std::uniform_real_distribution<float> x_dist{0,  + 2.4f };// range 0 - 2.2
    std::uniform_real_distribution<float> y_dist{-1.5f,  + 1.5f };


    // callbacks
    // where rrt actually happens
//    void pf_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    void pf_callback(const nav_msgs::Odometry::ConstPtr& pose_msg);
    // updates occupancy grid
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);


    void Update(float min_angle, float angle_inc);
    geometry_msgs::PointStamped transformFromLidar2Map(geometry_msgs::PointStamped lidar_pt);
    geometry_msgs::PointStamped transformFromMap2Lidar(geometry_msgs::PointStamped map_pt);
    void visualise_markers( visualization_msgs::Marker marker, double x,double y, int id);
    int getRowFromPoint(double y);
    int getColFromPoint(double x);

    int rc_2_ind(int r, int c);
    void inflation(int row, int col);


    // RRT methods
    std::vector<double> sample(double& goal_x, double goal_y);
    int nearest(std::vector<Node> &tree, std::vector<double> &sampled_point);
    Node steer(Node &nearest_node, std::vector<double> &sampled_point, int& nearest_point_index);
    bool check_collision(Node &nearest_node, Node &new_node);
    bool is_goal(Node &latest_added_node, double & goal_x, double & goal_y);
    std::vector<Node> find_path(std::vector<Node> &tree, Node &latest_added_node);
    // RRT* methods
    double cost(std::vector<Node> &tree, Node &node);
    double line_cost(Node &n1, Node &n2);
    std::vector<int> near(std::vector<Node> &tree, Node &node);

};

