#include <iostream>
#include <fstream>

#include <eigen3/Eigen/QR>
#include <vector>


#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf/LinearMath/Vector3.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/LinearMath/Matrix3x3.h"
#include <visualization_msgs/Marker.h>

#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "race/drive_param.h"

struct mpc_ref_t {
    float x, y;
    float phi;
    float speed;
};

struct speed_pos_t {
    float x, y;
    ros::Time t;
};



#include "mpc.cpp"

geometry_msgs::Pose new_pose(float x, float y, float yaw, float speed) {

        geometry_msgs::Pose p;
        p.position.x = x;
        p.position.y = y;
        p.position.z = 0;

        //get quaternion
        tf::Quaternion q(0, 0, yaw);
        //q *=speed;
        p.orientation.x = q.getX();
        p.orientation.y = q.getY();
        p.orientation.z = q.getZ();
        p.orientation.w = q.getW();

        return p;
}


// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}


float point_dst(float x1, float y1, float x2, float y2) {

    float dx = x1 -x2;
    float dy = y1 -y2;
    return sqrt(dx*dx + dy*dy);
}


bool point_passed(float car_x, float car_y, float x, float y, double angle) {

    float dst1 = point_dst(car_x, car_y, x, y);

    car_x += cos(angle)*dst1;
    car_y += sin(angle)*dst1;

    float dst2 = point_dst(car_x, car_y, x, y);

    if(dst2 > dst1)
        return true;
    return false;
}

visualization_msgs::Marker init_point_viz(const char *frame, float r, float g, float b) {

    visualization_msgs::Marker m;
    m.header.stamp = ros::Time::now();
    m.header.frame_id = frame;
    m.ns= "spheres";
    m.action= visualization_msgs::Marker::ADD;
    m.id = 0;
    m.type = visualization_msgs::Marker::SPHERE_LIST;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 1.0;
    return m;
}


int main(int argc, char **argv) {


    ros::init(argc, argv, "mpc");
    ros::NodeHandle n;
    //ros::Subscriber sub = n.subscribe("scan", 1, callback);
    ros::Publisher points_pub = n.advertise<visualization_msgs::Marker>("ref_points", 10);
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("ref_path", 10);
    ros::Publisher ref_pub = n.advertise<visualization_msgs::Marker>("ref_horizon", 10);
    ros::Publisher calc_pub = n.advertise<visualization_msgs::Marker>("calc_horizon", 10);
    ros::Publisher drive_pub = n.advertise<race::drive_param>("drive_parameters", 1);

    // DEFINE POINTS REF PATH
    visualization_msgs::Marker ref_points_msg = init_point_viz("map", 1, 0, 0);


    std::ifstream path_file("path.txt");
    if (!path_file.is_open()) {
        std::cout<<"cant open path\n";
        return 1;
    }
    int PTS_DIM = 0;
    path_file>>PTS_DIM;
    std::cout<<"PATH DIM: "<<PTS_DIM<<"\n";
    geometry_msgs::Pose *pts = new geometry_msgs::Pose[PTS_DIM];

    for(int i=0; i<PTS_DIM; i++) {
        float x, y;
        path_file>>x>>y; 
        pts[i] = new_pose(x,y, 0, 10);

        geometry_msgs::Point p;
        p.x = pts[i].position.x;
        p.y = pts[i].position.y;
        p.z = 0;
        ref_points_msg.points.push_back(p);
    }
    path_file.close();

    // FOR SPEED CALCULATION
    const int SPEED_POS = 5;
    speed_pos_t speed_pos[SPEED_POS];
    float speed = 0;
    int speed_pos_i = 0;

    // PATH ITER POS
    int POS = PTS_DIM;

    tf::TransformListener listener;
    ros::Rate rate(40.0);
    while(n.ok()) {

        // GET ROBOT POSITION    
        tf::StampedTransform transform;
        try{
            listener.lookupTransform("/map", "/footprint",  
                                    ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        float car_x = transform.getOrigin().x();
        float car_y = transform.getOrigin().y();
        double yaw, pitch, roll;
        tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);

        //UPDATE speed
        float speed_calc = 0; //speed for mean
        for(int i=0; i<SPEED_POS; i++) {
            speed_pos_t pos = speed_pos[(speed_pos_i +i) % SPEED_POS ];
            float dx = car_x - pos.x;
            float dy = car_y - pos.y;
            float dt = (transform.stamp_ - pos.t).toSec();
            float space = sqrt(dx*dx + dy*dy);
            speed_calc += space / dt;
        }
        speed_calc /= SPEED_POS;
        if(speed_calc == speed_calc)
            speed = speed_calc;
        std::cout<<"speed: "<<speed<<"\n";
        // insert pos in array
        speed_pos[speed_pos_i % SPEED_POS].x = car_x;
        speed_pos[speed_pos_i % SPEED_POS].y = car_y;
        speed_pos[speed_pos_i % SPEED_POS].t = transform.stamp_;
        speed_pos_i++;

        //UPDATE ITER POS
        if(point_passed(car_x, car_y, pts[POS % PTS_DIM].position.x, pts[POS % PTS_DIM].position.y, yaw)) {
            POS += 1;
            std::cout<<"POS: "<<POS % PTS_DIM<<"\n";
        }
    
        // CALCULATE RELATIVE PATH
        int PTS4PATH = 15;
        int BEHIND = 5;

        Eigen::VectorXd waypoints_x_eig(PTS4PATH);
        Eigen::VectorXd waypoints_y_eig(PTS4PATH);
        for(int i=0; i<PTS4PATH; i++) {
            geometry_msgs::PoseStamped pIN;
            pIN.header.frame_id = "map";
            pIN.pose = pts[(POS + i - BEHIND) % PTS_DIM];
            geometry_msgs::PoseStamped pOUT;
            pOUT.header.frame_id = "footprint";

            listener.transformPose("footprint", pIN, pOUT);
            waypoints_x_eig(i) = pOUT.pose.position.x;
            waypoints_y_eig(i) = pOUT.pose.position.y;
        }
        auto coeffs = polyfit(waypoints_x_eig, waypoints_y_eig, 3);

        nav_msgs::Path ref_path_msg;
        ref_path_msg.header.stamp = ros::Time::now();
        ref_path_msg.header.frame_id = "footprint";
        const int PATH_DIM = 200;
        geometry_msgs::PoseStamped path[PATH_DIM];
        
        float diff = (waypoints_x_eig(PTS4PATH-1) - waypoints_x_eig(0))/PATH_DIM;
        for(int i=0; i<PATH_DIM; i++) {
            float x =  waypoints_x_eig(0) + diff*i;
            float y = polyeval(coeffs, x);
            geometry_msgs::PoseStamped pS;
            pS.pose = new_pose(x, y, 0, 10);
            path[i] = pS;
        }
        std::vector<geometry_msgs::PoseStamped> v2(path, path+(PATH_DIM));
        ref_path_msg.poses = v2; 

        // REFS
        float A = coeffs[3];
        float B = coeffs[2];
        float C = coeffs[1];
        float D = coeffs[0];
        const int INTERVALS = 10;
        mpc_ref_t mpc_ref[INTERVALS];
        race::drive_param drive_msg = mpc_control(A, B, C, D, speed, mpc_ref);
	    printf("steer: %f, throttle: %f\n", drive_msg.angle, drive_msg.velocity);

        visualization_msgs::Marker calc_horizon_msg = init_point_viz("footprint", 0, 0, 1);
        for(int i=0; i<INTERVALS; i++) {
            geometry_msgs::Point p;
            p.x = mpc_ref[i].x;
            p.y = mpc_ref[i].y;
            calc_horizon_msg.points.push_back(p);
        }

        drive_pub.publish(drive_msg);
        points_pub.publish(ref_points_msg);
        path_pub.publish(ref_path_msg);
        calc_pub.publish(calc_horizon_msg);

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}