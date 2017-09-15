#include <iostream>
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

    // DEFINE POINTS REF PATH
    visualization_msgs::Marker ref_points_msg = init_point_viz("map", 1, 0, 0);

    const int PTS_DIM = 21;
    geometry_msgs::Pose pts[PTS_DIM];
    pts[ 0] = new_pose( 0.0,  0.0, 0, 10);
    pts[ 1] = new_pose( 1.0,  0.0, 0, 10);
    pts[ 2] = new_pose( 3.0,  1.0, 0, 10);
    pts[ 3] = new_pose( 3.0,  3.0, 0, 10);
    pts[ 4] = new_pose( 4.0,  5.0, 0, 10);
    pts[ 5] = new_pose( 6.0,  6.5, 0, 10);
    pts[ 6] = new_pose( 7.0,  8.0, 0, 10);
    pts[ 7] = new_pose( 6.0, 10.0, 0, 10);
    pts[ 8] = new_pose( 4.0, 10.8, 0, 10);
    pts[ 9] = new_pose( 2.0, 11.0, 0, 10);
    pts[10] = new_pose( 0.0, 10.0, 0, 10);
    pts[11] = new_pose(-1.0,  8.0, 0, 10);
    pts[12] = new_pose(-2.0,  6.0, 0, 10);
    pts[13] = new_pose(-4.0,  5.0, 0, 10);
    pts[14] = new_pose(-6.0,  4.0, 0, 10);
    pts[15] = new_pose(-7.0,  3.0, 0, 10);
    pts[16] = new_pose(-8.0,  2.0, 0, 10);
    pts[17] = new_pose(-7.0,  1.0, 0, 10);
    pts[18] = new_pose(-6.0,  0.0, 0, 10);
    pts[19] = new_pose(-4.0,  0.0, 0, 10);
    pts[20] = new_pose(-2.0,  0.0, 0, 10);
    for(int i=0; i<PTS_DIM; i++) {
        geometry_msgs::Point p;
        p.x = pts[i].position.x;
        p.y = pts[i].position.y;
        p.z = 0;
        ref_points_msg.points.push_back(p);
    }

    // PATH ITER POS
    int POS = 0;

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

        //UPDATE ITER POS
        if(point_passed(car_x, car_y, pts[POS+2].position.x, pts[POS+2].position.y, yaw)) {
            POS += 1;
            std::cout<<"POS: "<<POS<<"\n";
        }
    
        // CALCULATE RELATIVE PATH
        int PTS4PATH = 5;

        Eigen::VectorXd waypoints_x_eig(PTS4PATH);
        Eigen::VectorXd waypoints_y_eig(PTS4PATH);
        for(int i=0; i<PTS4PATH; i++) {
            geometry_msgs::PoseStamped pIN;
            pIN.header.frame_id = "map";
            pIN.pose = pts[(POS + i) % PTS_DIM];
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

        // CALCULATE REFS
        visualization_msgs::Marker ref_horizon_msg = init_point_viz("footprint", 0, 1, 0);
        float SECS = 5.0;
        int INTERVALS = 10;
        float L = 3.0;
        float dt = SECS/INTERVALS;
        float x = 0;
        for(int i=0; i<INTERVALS; i++) {
            geometry_msgs::Point p;
            float phi = atan(3*coeffs[3]*x*x + 2*coeffs[2]*x + coeffs[1]);
            x += cos(phi)*dt*2.0; //1.0 is speed
            p.x = x;
            p.y = polyeval(coeffs, p.x);
            ref_horizon_msg.points.push_back(p);
        }

        points_pub.publish(ref_points_msg);
        path_pub.publish(ref_path_msg);
        ref_pub.publish(ref_horizon_msg);

        mpc_control();

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}