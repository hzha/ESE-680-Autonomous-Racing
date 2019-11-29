
#include "rrt/rrt.h"
using namespace std;
// Destructor of the RRT class
RRT::~RRT() {
    ROS_INFO("RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(ros::NodeHandle &nh): nh_(nh), tfListener(tfBuffer), gen((std::random_device())()) {

    // TODO: Load parameters from yaml file, you could add your own parameters to the rrt_params.yaml file
    std::string pose_topic, scan_topic,map_topic;
    pose_topic = "/pf/pose/odom";
    scan_topic = "/scan";
    map_topic = "/map_metadata";
    nh_.getParam("pose_topic", pose_topic);
    nh_.getParam("scan_topic", scan_topic);
    nh_.getParam("map_topic",map_topic );
    nh_.getParam("lookahead_wp", lookahead_wp);
    nh_.getParam("lookahead_path", lookahead_path);
    nh_.getParam("inflation_size",inflation_size);
    nh_.getParam("step_size", step_size);
    nh_.getParam("goal_threshold", goal_threshold);
    nh_.getParam("MAX_ITER", MAX_ITER);
    nh_.getParam("interpolation_number",interpolation_number);
    nh_.getParam("sample_goal_prob_reciprocal",sample_goal_prob_reciprocal);
    nh_.getParam("Kp",Kp);
    nh_.getParam("map_clear_pubnumber",map_clear_pubnumber);


    // ROS publishers
    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/occumap",10,true);
    drive_pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("/nav", 10);
    vis_pub = nh_.advertise<visualization_msgs::Marker>("/waypoints_marker", 10);
    tree_line_pub = nh_.advertise<visualization_msgs::Marker>("/tree_line", 10);
    path_line_pub = nh_.advertise<visualization_msgs::Marker>("/path_line", 10);

    // ROS subscribers
    scan_sub_ = nh_.subscribe(scan_topic, 10, &RRT::scan_callback, this);
    pf_sub_ = nh_.subscribe(pose_topic, 10, &RRT::pf_callback, this);

   boost::shared_ptr<nav_msgs::MapMetaData const> map_ptr;
   nav_msgs::MapMetaData map_msg;

   map_ptr = ros::topic::waitForMessage<nav_msgs::MapMetaData>(map_topic);
   if(map_ptr != NULL){
       map_msg = *map_ptr;
   }


   levine_map_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map");

    occumap.data = levine_map_ptr->data;
    occumap.header.frame_id  = "map";
    occumap.header.stamp = ros::Time::now();
    occumap.info.height =  map_msg.height;
    occumap.info.origin = map_msg.origin;
    occumap.info.width = map_msg.width;
    occumap.info.resolution = map_msg.resolution;
    occumap.info.map_load_time = map_msg.map_load_time;

    ROS_INFO("Created new RRT Object.");

    //visualize waypoint

    std::ifstream file("/home/hunter/hongtaozhang_ws/src/ESE-680-Autonomous-Racing/rrt/waypoints.csv");

    while(file.good()){
        getline(file, temp, ',');
        waypoint_x_data.push_back(stod(temp));
        getline(file, temp, '\n');
        waypoint_y_data.push_back(stod(temp));
    }

    file.close();

    tree_line_marker.header.frame_id = "map";
    tree_line_marker.ns = "tree_line_vis";
    tree_line_marker.type = visualization_msgs::Marker::LINE_LIST;
    tree_line_marker.action = visualization_msgs::Marker::ADD;
    tree_line_marker.lifetime = ros::Duration(0.05);
    tree_line_marker.pose.position.z = 0;

    tree_line_marker.scale.x = 0.03;
    tree_line_marker.scale.y = 0.03;
    tree_line_marker.scale.z = 0.03;
    tree_line_marker.color.a = 1.0;
    tree_line_marker.color.r = 1.0;
    tree_line_marker.color.g = 0.0;
    tree_line_marker.color.b = 0.0;
    tree_line_marker.id = 0;

    path_line_marker.header.frame_id = "map";
    path_line_marker.ns = "path_line_vis";
    path_line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_line_marker.action = visualization_msgs::Marker::ADD;
    path_line_marker.lifetime = ros::Duration(0.05);
    path_line_marker.pose.position.z = 0;

    path_line_marker.scale.x = 0.05;
    path_line_marker.scale.y = 0.05;
    path_line_marker.scale.z = 0.05;
    path_line_marker.color.a = 1.0;
    path_line_marker.color.r = 0.0;
    path_line_marker.color.g = 1.0;
    path_line_marker.color.b = 0.0;
    path_line_marker.id = 0;

}



void RRT::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
    // The scan callback, update occupancy grid here
    // Args:
    //    scan_msg (*LaserScan): pointer to the incoming scan message

    range_laser = scan_msg->ranges;
    float min_angle = scan_msg->angle_min;
    float angle_inc = scan_msg->angle_increment;

    Update(min_angle,angle_inc);

    map_pub_.publish(occumap);
    map_erase_count++;

    if(map_erase_count%map_clear_pubnumber == 0 ){
    map_erase_count = 0;
    occumap.data = levine_map_ptr->data;
    }


}


void RRT::pf_callback(const nav_msgs::Odometry::ConstPtr &pose_msg) {
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
    // Returns:
    //
    // tree as std::vector
    std::vector<Node> tree;
    std::vector<Node> path;
    Node start_Node;

    double goal_x =0;//in global frame
    double goal_y =0;

    start_Node.x = (pose_msg->pose).pose.position.x;
    start_Node.y = (pose_msg->pose).pose.position.y;
    start_Node.is_root = true;
    start_Node.cost = 0;
    tree.push_back(start_Node);

    tf::Quaternion q(
            (pose_msg->pose).pose.orientation.x,
            (pose_msg->pose).pose.orientation.y,
            (pose_msg->pose).pose.orientation.z,
            (pose_msg->pose).pose.orientation.w);

    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    std::vector<double> x_car_wp(waypoint_x_data.size(), 0);//path point coordinate in laser frame
    std::vector<double> y_car_wp(waypoint_y_data.size(), 0);

     geometry_msgs::PointStamped map_wypt;
      map_wypt.header.frame_id = "map";
      map_wypt.point.z = 0;

      geometry_msgs::PointStamped lidar_wypt;
      lidar_wypt.header.frame_id = "laser";
      lidar_wypt.point.z = 0;

    for(int i = 0; i<waypoint_y_data.size(); i++) {

         map_wypt.point.x = waypoint_x_data[i];
         map_wypt.point.y = waypoint_y_data[i];

         lidar_wypt = transformFromMap2Lidar(map_wypt);

         x_car_wp[i] = lidar_wypt.point.x;//delta_x * cos(yaw) + delta_y * sin(yaw);

         y_car_wp[i] = lidar_wypt.point.y;//-delta_x * sin(yaw) + delta_y * cos(yaw);
    }

    //calculate the distances between the car and all waypoints
    // and find the closest one relative to the look ahead distance.
    min_gap_wp = DBL_MAX;//initialize min_gap

    for(int i = 0; i<waypoint_y_data.size(); i++){

        double dist = pow(pow(x_car_wp[i],2) + pow(y_car_wp[i],2),0.5);
        double gap = fabs(lookahead_wp - dist);
        // the closest is set to inf initially
       // ROS_INFO("car wp x: %f", x_car_wp[i]);
        if (min_gap_wp > gap && x_car_wp[i] >= 0.0 ) {
            if(occumap.data[rc_2_ind(getRowFromPoint(waypoint_y_data[i]),getColFromPoint(waypoint_x_data[i]))] == 0)
            {
                min_gap_wp = gap;
                //ROS_INFO("n: %d", n);
                n= i;
            }
        }
    }
    goal_x = waypoint_x_data[n];
    goal_y = waypoint_y_data[n];
   // ROS_INFO("goal_x: %f, goal_y: %f", goal_x, goal_y);

    visualization_msgs::Marker marker_goal;
    marker_goal.ns = "goal_vis";
    marker_goal.color.r = 1.0;tree_line_marker.lifetime = ros::Duration(0.05);
    marker_goal.color.g = 0.0;
    marker_goal.color.b = 0.0;
    visualise_markers(marker_goal, goal_x,goal_y,1);
    std::vector<int> neighbour;
    double new_cost ;



    for(int i = 0; i < MAX_ITER; i++) {

        std::vector<double> sample_point = sample(goal_x, goal_y);

        int nearest_point_ind = nearest(tree, sample_point);
        Node new_point = steer(tree[nearest_point_ind], sample_point, nearest_point_ind);
        bool collision = check_collision(tree[nearest_point_ind], new_point);

        if (collision == true) {
            continue;
        }
        else {
            new_point.cost = cost(tree,new_point);
            neighbour = near(tree, new_point);

            for(int j = 0; j < neighbour.size(); j++){

                new_cost = tree[neighbour[j]].cost + line_cost(new_point,tree[neighbour[j]]);
                if(new_cost < new_point.cost){
                    new_point.cost = new_cost;
                    new_point.parent = neighbour[j];

                }
            }
            // rewire the tree
            for(int i = 0; i< neighbour.size();i++){
                double neighbour_cost = new_point.cost + line_cost(tree[neighbour[i]], new_point);
                if(neighbour_cost < tree[neighbour[i]].cost){
                    tree[neighbour[i]].cost = neighbour_cost;
                    tree[neighbour[i]].parent = tree.size() ;

                }
            }
            tree.push_back(new_point);


            tree_line_marker.points.clear();
            geometry_msgs::Point p1;
            p1.x = tree[nearest_point_ind].x;
            p1.y = tree[nearest_point_ind].y;
            tree_line_marker.points.push_back(p1);
            geometry_msgs::Point p2;
            p2.x = new_point.x;
            p2.y = new_point.y;
            tree_line_marker.points.push_back(p2);
            tree_line_marker.id ++;
            tree_line_pub.publish(tree_line_marker);
            if( is_goal(new_point, goal_x, goal_y)){
                //ROS_INFO("path founded");
                path = find_path(tree,new_point);

                break;
            }

        }
    }

    // if no path found, use the last path
    if(path.size() == 0)
        path=path_old;

    path_old = path;

    geometry_msgs::Point p3;
    path_line_marker.points.clear();
    for (int i = 0; i < path.size(); i++) {
        p3.x = path[i].x;
        p3.y = path[i].y;
        path_line_marker.points.push_back(p3);

    }
    path_line_pub.publish(path_line_marker);

    //ROS_INFO("path size: %d", path.size());
    std::vector<double> x_car_path;
    std::vector<double> y_car_path;

    // path found as Path message
    visualization_msgs::Marker marker_path;
    marker_path.ns = "path_vis";
    marker_path.color.r = 0.0;
    marker_path.color.g = 0.0;
    marker_path.color.b = 1.0;

    geometry_msgs::PointStamped map_pathpt;
    map_pathpt.header.frame_id = "map";
    map_pathpt.point.z = 0;

    geometry_msgs::PointStamped lidar_pathpt;
    lidar_pathpt.header.frame_id = "laser";
    lidar_pathpt.point.z = 0;

    for (auto i=1; i<path.size(); ++i){

        map_pathpt.point.x = path[i].x;
        map_pathpt.point.y = path[i].y;
        lidar_pathpt = transformFromMap2Lidar(map_pathpt);

        visualise_markers(marker_path, path[i].x,path[i].y, i);

        x_car_path.push_back(lidar_pathpt.point.x);
        y_car_path.push_back(lidar_pathpt.point.y);
    }

    min_gap_path = DBL_MAX;
    //coordinate of closest path point in laser frame
    for (auto i=0; i<x_car_path.size(); ++i){
        dis = pow((x_car_path[i]*x_car_path[i] + y_car_path[i]*y_car_path[i]), 0.5);
        gap = fabs(dis-lookahead_path);
        if (gap < min_gap_path){// && (x_car_path[i]>=0.0)){
            k = i;
            min_gap_path = gap;

        }
    }

    visualization_msgs::Marker marker_chasing;
    marker_chasing.ns = "path_chasing";
    marker_chasing.color.r = 0.0;
    marker_chasing.color.g = 1.0;
    marker_chasing.color.b = 0.0;
    if(k>0)
    visualise_markers(marker_chasing, path[k+1].x, path[k+1].y,1 );


    //calculate arc radius gamma
    //std::cout << k << std::endl;
    x_d = x_car_path[k];
    y_d = y_car_path[k];
    L_d_square = x_d*x_d + y_d*y_d;
    gamma = 2*y_d/(L_d_square);
    //should have another variable:curvature = 1 / gamma;
    angle = Kp*gamma;//steering angle is proportional to the curvature
    angle = std::min(angle, 0.4189);
    angle = std::max(angle, -0.4189);

    if (fabs(angle*180/M_PI)<10){
        velocity = 4;
    }else if (fabs(angle*180/M_PI)<20){
        velocity = 2.5;
    }else{
        velocity = 2;
    }

    drive_msg.header.stamp = ros::Time::now();
    drive_msg.header.frame_id = "laser";
    drive_msg.drive.steering_angle = angle;
    drive_msg.drive.speed = velocity;
    drive_pub.publish(drive_msg);

}

void RRT::visualise_markers( visualization_msgs::Marker markers, double x,double y, int id){

    markers.header.frame_id = "map";
    markers.type = visualization_msgs::Marker::SPHERE;
    markers.action = visualization_msgs::Marker::ADD;
    markers.pose.position.z = 0;
    markers.pose.orientation.x = 0.0;
    markers.pose.orientation.y = 0.0;
    markers.pose.orientation.z = 0.0;
    markers.pose.orientation.w = 1.0;
    markers.scale.x = 0.1;
    markers.scale.y = 0.1;
    markers.scale.z = 0.1;
    markers.color.a = 1.0;
    markers.lifetime = ros::Duration(0.05);

    markers.id = id;
    markers.header.stamp = ros::Time::now();
    markers.pose.position.x = x;
    markers.pose.position.y = y;
    vis_pub.publish(markers);

}
//dynamic layer map update
void RRT::Update(float min_angle,float angle_inc){

    int array_size = 0;
    min_angle = min_angle + 60*M_PI/180;

    geometry_msgs::PointStamped lidar_pt;
    geometry_msgs::PointStamped map_pt;

    lidar_pt.header.frame_id = "laser";
    lidar_pt.point.z = 0;
    map_pt.header.frame_id = "map";
    map_pt.point.z = 0;

    for(std::vector<float>::iterator it = range_laser.begin() + 180; it != range_laser.end() - 180; ++it) {

        array_size++;
        double beam = (*it);

        if (std::isnan(*it) || std::isinf(*it)) {
            range_laser.erase(it);
            continue;
        }

        lidar_pt.point.x = beam* cos(min_angle + (array_size - 1) * angle_inc);
        lidar_pt.point.y = beam* sin(min_angle + (array_size - 1) * angle_inc);

        map_pt = transformFromLidar2Map(lidar_pt);

        int col = getColFromPoint(map_pt.point.x);
        int row = getRowFromPoint(map_pt.point.y);

        //occumap_dynamic.data[rc_2_ind(row,col)] = 100;
        if(beam < 6.0)
        inflation(row, col);

        }
}

int RRT::getRowFromPoint(double y){
    return static_cast<int>((y - occumap.info.origin.position.y) / occumap.info.resolution);
}

int RRT::getColFromPoint(double x){
    return static_cast<int>((x - occumap.info.origin.position.x) / occumap.info.resolution);
}

int RRT::rc_2_ind(int r, int c){
    return (r*occumap.info.width + c);
}

geometry_msgs::PointStamped RRT::transformFromLidar2Map(geometry_msgs::PointStamped lidar_pt){

    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::PointStamped map_pt;

    try {
        transformStamped = tfBuffer.lookupTransform("map", "laser", ros::Time(0));
        tf2::doTransform(lidar_pt, map_pt, transformStamped);

    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("Failure %s\n", ex.what());
        ros::Duration(1.0).sleep();
    }
    return map_pt;

}

geometry_msgs::PointStamped RRT::transformFromMap2Lidar(geometry_msgs::PointStamped map_pt){

    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::PointStamped lidar_pt;

    try {
        transformStamped = tfBuffer.lookupTransform("laser", "map", ros::Time(0));
        tf2::doTransform(map_pt, lidar_pt, transformStamped);

    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("Failure %s\n", ex.what());
        ros::Duration(1.0).sleep();
    }
    return lidar_pt;

}

void RRT::inflation(int row, int col){

    int inf_side = 2*(inflation_size)+1;

    std::vector<int> cols(inf_side,0);
    std::vector<int> rows(inf_side,0);

    int ctmp = col-inflation_size;
    int rtmp = row-inflation_size;

    for(int i =0; i< inf_side;i++){

        cols[i] = ctmp;
        ctmp ++;
        rows[i] = rtmp;
        rtmp ++;

    }

   int start =  0 ;
   int end = rows.size();

       for (int i = start; i < end; i++) {
           for (int j = start; j < end; j++) {

               occumap.data[rc_2_ind(rows[i], cols[j])] = 100;
           }
       }
   }

std::vector<double> RRT::sample(double& goal_x, double goal_y) {
    // This method returns a sampled point from the free space
    // Should restrict so that it only samples a small region
    // of interest around the car's current position
    // Args:
    // Returns:
    //sampled_point (std::vector<double>): size:2; the sampled point in free space

    std::vector<double> sampled_point;

    geometry_msgs::PointStamped lidar_pt;
    geometry_msgs::PointStamped map_pt;
    lidar_pt.header.frame_id = "laser";
    int free_indicator = 0;
    int prob_indi = 0;
    while(free_indicator == 0){
        prob_indi++;
        //with certain probability, the sample point is goal point
        if(prob_indi % sample_goal_prob_reciprocal == 0){
            lidar_pt.point.x = goal_x;
            lidar_pt.point.y = goal_y;
            lidar_pt.point.z = 0;
//            free_indicator = 1;
        }
        else{
            lidar_pt.point.x = x_dist(gen);
            lidar_pt.point.y = y_dist(gen);
            lidar_pt.point.z = 0;
        }

        map_pt = transformFromLidar2Map(lidar_pt);
        double x_g = map_pt.point.x;
        double y_g = map_pt.point.y;
        int index = rc_2_ind(getRowFromPoint(y_g),getColFromPoint(x_g));

        if(occumap.data[index] > 0){//free space
            continue;
        }

        sampled_point.push_back(x_g);
        sampled_point.push_back(y_g);
        free_indicator = 1;

    }

    return sampled_point;//
}

int RRT::nearest(std::vector<Node> &tree, std::vector<double> &sampled_point) {
    // This method returns the nearest node on the tree to the sampled point
    // Args:
    //     tree (std::vector<Node>): the current RRT tree
    //     sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //     nearest_node (int): index of nearest node on the tree
    int nearest_node = 0;
    float cur_dist;
    float min_dist = FLT_MAX;
    for(int i = 0; i < tree.size(); i++) {
        cur_dist = pow(tree[i].x - sampled_point[0], 2) + pow(tree[i].y - sampled_point[1], 2);
        if (cur_dist < min_dist) {
            nearest_node = i;
            min_dist = cur_dist;
        }
    }

    return nearest_node;
}

Node RRT::steer(Node &nearest_node, std::vector<double> &sampled_point, int& nearest_point_index) {
    // The function steer:(x,y)->z returns a point such that z is “closer” 
    // to y than x is. The point z returned by the function steer will be 
    // such that z minimizes ||z−y|| while at the same time maintaining 
    //||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

    // y is on the tree and x is the sampled point, z is the point along x-y direction which will
    // be added on the tree. basically, expand the tree towards the sample point (within a max dist)

    // Args:
    //    nearest_node (Node): nearest node on the tree to the sampled point
    //    sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //    new_node (Node): new node created from steering
    Node new_node;

    float dist = sqrt(pow(nearest_node.x - sampled_point[0], 2) + pow(nearest_node.y - sampled_point[1], 2));

    if(dist > step_size){
        new_node.x = (step_size / dist) * (sampled_point[0] - nearest_node.x) + nearest_node.x;

        new_node.y = (step_size / dist) * (sampled_point[1] - nearest_node.y) + nearest_node.y;
        new_node.is_root = false;
        new_node.parent = nearest_point_index;

    }
    else{
        new_node.x = sampled_point[0];
        new_node.y = sampled_point[1];
        new_node.is_root = false;
        new_node.parent = nearest_point_index;
    }
    return new_node;
}

bool RRT::check_collision(Node &nearest_node, Node &new_node) {
    // This method returns a boolean indicating if the path between the 
    // nearest node and the new node created from steering is collision free
    // Args:
    //    nearest_node (Node): nearest node on the tree to the sampled point
    //    new_node (Node): new node created from steering
    // Returns:
    //    collision (bool): true if in collision, false otherwise
    bool collision = false;
    for(int i = 0; i <= interpolation_number; i++) {
        float x = (float)i / (float) interpolation_number * (new_node.x - nearest_node.x) + nearest_node.x;
        float y = (float)i / (float) interpolation_number * (new_node.y - nearest_node.y) + nearest_node.y;

        int index = rc_2_ind(getRowFromPoint(y), getColFromPoint(x));
        if (occumap.data[index] > 0) {
            collision = true;
            break;
        }
    }
    return collision;
}// add interpolation_number

bool RRT::is_goal(Node &latest_added_node, double & goal_x, double & goal_y) {
    // This method checks if the latest node added to the tree is close
    // enough (defined by goal_threshold) to the goal so we can terminate
    // the search and find a path
    // Args:
    //   latest_added_node (Node): latest addition to the tree
    //   goal_x (double): x coordinate of the current goal
    //   goal_y (double): y coordinate of the current goal
    // Returns:
    //   close_enough (bool): true if node close enough to the goal

    bool close_enough = false;
    float dist = sqrt(pow(latest_added_node.x - goal_x, 2) + pow(latest_added_node.y - goal_y, 2));

    close_enough = (dist > goal_threshold) ? false : true;
    return close_enough;
}

std::vector<Node> RRT::find_path(std::vector<Node> &tree, Node &latest_added_node) {
    // This method traverses the tree from the node that has been determined
    // as goal
    // Args:
    //   latest_added_node (Node): latest addition to the tree that has been
    //      determined to be close enough to the goal
    // Returns:
    //   path (std::vector<Node>): the vector that represents the order of
    //      of the nodes traversed as the found path

    std::vector<Node> found_path;

    found_path.push_back(latest_added_node);
    int i = tree.size() - 1;
    while(i != 0){

        found_path.push_back(tree[tree[i].parent]);
        i = tree[i].parent;

    }
    std::reverse(found_path.begin(),found_path.end());
    return found_path;
}

// RRT* methods
double RRT::cost(std::vector<Node> &tree, Node &node) {
    // This method returns the cost associated with a node
    // Args:
    //    tree (std::vector<Node>): the current tree
    //    node (Node): the node the cost is calculated for
    // Returns:
    //    cost (double): the cost value associated with the node

    double cost = tree[node.parent].cost + line_cost(node, tree[node.parent]);
    return cost;
}

double RRT::line_cost(Node &n1, Node &n2) {
    // This method returns the cost of the straight line path between two nodes
    // Args:
    //    n1 (Node): the Node at one end of the path
    //    n2 (Node): the Node at the other end of the path
    // Returns:
    //    cost (double): the cost value associated with the path

    double cost = 0;
    cost = pow(pow(n1.x - n2.x, 2) + pow(n1.y - n2.y,2) , 0.5);

    return cost;
}

std::vector<int> RRT::near(std::vector<Node> &tree, Node &node) {
    // This method returns the set of Nodes in the neighborhood of a 
    // node.
    // Args:
    //   tree (std::vector<Node>): the current tree
    //   node (Node): the node to find the neighborhood for
    // Returns:
    //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood

    std::vector<int> neighborhood;
    double current_dist;
    //ROS_INFO("nearset_ dist: %f", sqrt(pow(tree[nearest_node].x - node.x,2)+ pow(tree[nearest_node].y - node.y,2)));
    for(int i = 0; i< tree.size(); i++){
        current_dist = pow(pow(tree[i].x - node.x,2)+ pow(tree[i].y - node.y,2),0.5);
        if(current_dist <= 1e-6)
            continue;
        if(current_dist <= near_range){
            //ROS_INFO("neighbour_dist: %f", current_dist);
            neighborhood.push_back(i);
        }

    }

    return neighborhood;
}
