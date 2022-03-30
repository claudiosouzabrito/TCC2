

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
//#include "sensor_msgs/Image.h"
//#include "sensor_msgs/PointCloud2.h"
//#include "sensor_msgs/LaserScan.h"
#include <string>
// #include "ekf.h"

#include "robot.h"
// #include "astar.h"
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>



using std::vector;
using std::string;
using namespace ros;
using namespace Eigen;
using namespace std;


class Vout{
    public:
        Vout(): v_out(0), w_out(0) {}
        Vout(double v_out, double w_out): v_out(v_out), w_out(w_out) {}

        double v_out;
        double w_out;

};

class NMPC{
    private:
        
        int mysign(double x);
        //double CostFunction(TRobotStateSim Robot, TTargetStateSim Target, Matrix2d& Ut);
        Matrix<double,2,8> calcUSteps(Matrix2d& Ud);
        Matrix2d saturate(Matrix2d& u);

    //    ros::Subscriber vision;
    
    public:
        NodeHandle node_;
        Subscriber ekf_estim;
        Subscriber Rob_vel;
        Subscriber cloud_pose;

        NMPC(){
            //subscreve valores do EKF
            //ekf_estim = node_.subscribe("/imu", 1, &NMPC::OdomCallback, this);

            //subscreve valores de velocidade do robo
            Rob_vel = node_.subscribe("/odom", 1, &NMPC::OdomCallback, this);
            cloud_pose = node_.subscribe("/particlecloud", 1, &NMPC::CloudCallback, this);
            
            //vision = node_.subscribe("/camera/rgb/image_raw",10, &NMPC::ImageCallback,this);
        }
        ~NMPC(){}

        Vout velocity;    
        MyRobot iRobot;
        MyCloud cloud;

        Vout NMPController(MyRobot iRobot, Trajectory Traj);
         
        void OdomCallback(const nav_msgs::Odometry::ConstPtr& vel);
        void CloudCallback(const geometry_msgs::PoseArray& pose);

        //void VelCallback(const nav_msgs::Odometry::ConstPtr& vel);
        double CostFunction(TRobotStateSim Robot, TTargetStateSim Target, Matrix2d& Ut);
        double DiffAngle(double a1, double a2);

        //void ImageCallback(const sensor_msgs::Image img)
        
};


