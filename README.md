# ESE-680-Autonomous-Racing
This is for the ESE-680-Autonomous-Racing. The ros lab is for basic message sending and receiving. And the safety_lab is for 
creating a node of emergency braking when the car is too close to an obstacle.

informed_RRT*

informed_RRT* algorithm in simulator with visualisation of tree, path and waypoints. The car could receive data from the static map and at the meantime constructing dynanic map with message from lidar scan. 

And the car gets its real-time position and pose in map by particle filter.

![informed_RRT*](https://user-images.githubusercontent.com/53478662/70093546-a4d45500-15ee-11ea-8916-70bae371d3c6.gif)

REACTIVE OBSTACLE AVOIDANCE

Reactive_method is utilized for reactive obstacle avoidance and maintaining high speed meanwhile. 

![reactive](https://user-images.githubusercontent.com/53478662/69002994-78b79500-08c8-11ea-8eb8-2853db54a6e4.gif)

SCAN MATCHING

Replicate the Point-to-Line-Iterative-Closest-Point(PLICP) algorithm. Match the first time and secong time lidar scan to 
realise the car's localization.

![scan_mactching](https://user-images.githubusercontent.com/53478662/69003358-2bd6bd00-08ce-11ea-8d81-60701f0f48a2.gif)








