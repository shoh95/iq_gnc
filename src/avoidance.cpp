#include <iostream>
#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <sensor_msgs/LaserScan.h>
#include <gnc_functions.hpp>


float targetloc_x = 0., targetloc_y = 70., targetloc_z = 2., targetloc_psi = 0.;
// float targetloc_lat, targetloc_lon, targetloc_alt, targetloc_heading;
// static float delta_x, delta_y, delta_z;
int mode_change = 0;


void scan_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    
    sensor_msgs::LaserScan current_2D_scan;
    current_2D_scan = *msg;
    float avoidance_vector_x = 0; 
    float avoidance_vector_y = 0;
    float attractive_vector = 0;
    float attractive_vector_x = 0;
    float attractive_vector_y = 0;
    bool avoid = false;
    
    // targetloc_lat = -35.362827;
    // targetloc_lon = 149.165158;
    // targetloc_alt = 2.;
    // targetloc_heading = 0.;
    float total_vector_x = 0;
    float total_vector_y = 0;
    
    for(int i=1; i<current_2D_scan.ranges.size(); i++)
    {
        
        float d0 = 4; 
        float k = 0.5;
        
        

        if(current_2D_scan.ranges[i] < d0 && current_2D_scan.ranges[i] > 3.5)
        {
            avoid = true;
            float x = cos(current_2D_scan.angle_increment*i);
            float y = sin(current_2D_scan.angle_increment*i);
            float U = -.5*k*pow(((1/current_2D_scan.ranges[i]) - (1/d0)), 2);   

            avoidance_vector_x = avoidance_vector_x + x*U;
            avoidance_vector_y = avoidance_vector_y + y*U;

        }
    }
    float k_att = 0.015;
    float current_heading = get_current_heading();
    float deg2rad = (M_PI/180);
    avoidance_vector_x = avoidance_vector_x*cos((current_heading)*deg2rad) - avoidance_vector_y*sin((current_heading)*deg2rad);
    avoidance_vector_y = avoidance_vector_x*sin((current_heading)*deg2rad) + avoidance_vector_y*cos((current_heading)*deg2rad);
    geometry_msgs::Point current_pos;
    current_pos = get_current_location();
    geometry_msgs::PoseStamped vertiport_pos;
    vertiport_pos = get_vertiport_location();

    float d_att_sqaure = pow((targetloc_x - current_pos.x), 2) + pow((targetloc_y - current_pos.y), 2);       // ADD Z components
    attractive_vector = 0.5*k_att*d_att_sqaure;
    attractive_vector_x = attractive_vector * sin((current_heading)*deg2rad);
    attractive_vector_y = attractive_vector * cos((current_heading)*deg2rad);

    if( sqrt(pow(attractive_vector_x,2) + pow(attractive_vector_y,2)) > 2.)
        {
            attractive_vector_x = 2 * (attractive_vector_x/sqrt(pow(attractive_vector_x,2) + pow(attractive_vector_y,2)));
            attractive_vector_y = 2 * (attractive_vector_y/sqrt(pow(attractive_vector_x,2) + pow(attractive_vector_y,2)));
        }

    total_vector_x = total_vector_x + attractive_vector_x;
    total_vector_y = total_vector_y + attractive_vector_y;

    set_destination(total_vector_x + current_pos.x, total_vector_y + current_pos.y, 2, 0);

    // delta_x = abs(get_current_location().x - targetloc_x);
    // delta_y = abs(get_current_location().y - targetloc_y);
    // delta_z = abs(get_current_location().z - targetloc_z);


    if(avoid)
    {
        if( sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)) > 2.)
        {
            avoidance_vector_x = 2 * (avoidance_vector_x/sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)));
            avoidance_vector_y = 2 * (avoidance_vector_y/sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)));
        }
        if( sqrt(pow(attractive_vector_x,2) + pow(attractive_vector_y,2)) > 2.)
        {
            attractive_vector_x = 2 * (attractive_vector_x/sqrt(pow(attractive_vector_x,2) + pow(attractive_vector_y,2)));
            attractive_vector_y = 2 * (attractive_vector_y/sqrt(pow(attractive_vector_x,2) + pow(attractive_vector_y,2)));
        }
        total_vector_x = attractive_vector_x + avoidance_vector_x;
        total_vector_y = attractive_vector_y + avoidance_vector_y;
        //geometry_msgs::Point current_pos;
        current_pos = get_current_location();
        set_destination(total_vector_x + current_pos.x, total_vector_y + current_pos.y, 2, 0);
    }


}

int main(int argc, char **argv)
{
    //initialize ros 
    ros::init(argc, argv, "gnc_node");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/darknet_ros/bounding_boxes", 1, scan_cb);
    ros::Subscriber collision_sub = n.subscribe<sensor_msgs::LaserScan>("/spur/laser/scan", 1, scan_cb);
    //initialize control publisher/subscribers
    init_publisher_subscriber(n);

    // wait for FCU connection
    wait4connect();

    //wait for used to switch to mode GUIDED
    wait4start();

    //create local reference frame 
    initialize_local_frame();

    //set speed
    set_speed(1.0);

    //request takeoff
    takeoff(2);

    // std::vector<gnc_api_waypoint> waypointList;
    // gnc_api_waypoint nextWayPoint;
    // nextWayPoint.x = targetloc_x;
    // nextWayPoint.y = targetloc_y;
    // nextWayPoint.z = targetloc_z;
    // nextWayPoint.psi = targetloc_psi;
    // waypointList.push_back(nextWayPoint);

    

    set_vertiport(targetloc_x, targetloc_y, targetloc_z, targetloc_psi);

    //std::printf("dx: %f, dy: %f\n", delta_x, delta_y);

    

    //specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
    ros::Rate rate(2.0);
    int counter = 0;
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();

        // if(check_vertiport_reached(0.3) == 0)
        // {
            
        //     set_vertiport(targetloc_x, targetloc_y, targetloc_z, targetloc_psi);
        // }

        if(check_vertiport_reached(0.3) == 1)
        {
            land();
        }

        
        
    
    }
    return 0;
}