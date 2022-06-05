//CALLBACK NA MAIN


#include "../include/nmpc.h"
#include <gazebo_msgs/GetModelState.h>
#include <math.h>
#include <numeric>
#include <ros/ros.h>
#include <fstream>

using namespace std;
using namespace ros;
using namespace gazebo_msgs;
int MAP_X = 212;
int MAP_Y = 442;

vector<double> CalcTetaVW(double Vx, double aX, double Vy, double aY){
    //double tetaRef_ = atan(Vy/Vx) * (180/PI);
    double Vref_ = sqrt(pow(Vx, 2)+pow(Vy, 2));
    double Wref_ = ((Vx*aY)-(Vy*aX))/(pow(Vx, 2)+pow(Vy, 2));

    vector<double> p = {Vref_, Wref_};
    return p;
}

int lerArquivo(string path, vector<double> *xref, vector<double> *yref){
    xref->clear();
    yref->clear();
    ifstream file;
    file.open(path);
    string n;
    int i = 0, num;
    while(getline(file, n)){
        if(i == 0){
            num = atoi(n.c_str());
        }
        else if(i <= num){
            xref->push_back(10.645*stod(n)/MAP_X);
        }
        else{
            yref->push_back(22.285*(MAP_Y - stod(n))/MAP_Y);
        }
        i++;
    }
    
    return num;
}

int main(int argc, char** argv){
  
    ros::init(argc, argv, "control_teste");
    NodeHandle no; // apagar?

    double mapx = 10.645;
    double mapy = 22.285;

    vector<double> xref;
    vector<double> yref;
    vector<double> x;
    vector<double> y;

    //coords1 = linha reta horizontal
    int num = lerArquivo("src/robot_control/mapas/coords.txt", &xref, &yref); 

    double Vx[xref.size()];
    double Vy[xref.size()];
    adjacent_difference(xref.begin(), xref.end(), Vx);
    adjacent_difference(yref.begin(), yref.end(), Vy);
    Vx[0] = 0;
    Vy[0] = 0;

    double acelX[xref.size()];
    double acelY[xref.size()];
    adjacent_difference(Vx, Vx + xref.size(), acelX);
    adjacent_difference(Vy, Vy + xref.size(), acelY);
    acelX[0] = 0;
    acelY[0] = 0;

    cout << "########## COMECO DA LOCOMOCAO ############" << endl;
    
    Trajectory traj;
    traj.v_ref = 0;
    traj.w_ref = 0;
    Rate loop_rate1(2.6);
    vector<double> VW = {0.0, 0.0};
    NMPC nmpc = NMPC();
    Publisher velPub = nmpc.node_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ServiceClient position_client = no.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    GetModelState objstate;
    geometry_msgs::Pose pose_real;



    // while(ros::ok){
    //     spinOnce();
    //     cout << "iRobot.x = " << nmpc.iRobot.x_rob << ", iRobot.y = " << nmpc.iRobot.y_rob << endl;
    //     cout << "ekf.x = " << nmpc.ekf.x << ", ekf.y = " << nmpc.ekf.y << endl;
    //     cout << "amcl.x = " << nmpc.cloud.x << ", amcl.y = " << nmpc.cloud.y << endl;
    //     loop_rate1.sleep();
                

    //     cout << endl;
    // }
    objstate.request.model_name = "my_robot";
    //objstate.request.relative_entity_name = "my_robot";

    for(int i = 0; i < xref.size(); i++) {
        spinOnce();
        cout << endl;

        cout << i << endl;
        traj.x_ref = xref[i];
        traj.y_ref = yref[i];
        traj.vx_ref = Vx[i];
        traj.vy_ref = Vy[i];

        VW = CalcTetaVW(Vx[i], acelX[i], Vy[i], acelY[i]);

        traj.v_ref = VW[0];
        traj.w_ref = VW[1];

        nmpc.velocity = nmpc.NMPController(nmpc.ekf, nmpc.cloud, nmpc.iRobot, traj);
        

        geometry_msgs::Twist msg;
        msg.linear.x = nmpc.velocity.v_out;
        msg.angular.z = nmpc.velocity.w_out;

        if(position_client.call(objstate)){
            pose_real = objstate.response.pose;
        }

        cout << "Querendo ir para X = " << traj.x_ref << ", Y = " << traj.y_ref << endl;
        cout << "Gazebo.x = " << pose_real.position.x << ", Gazebo.y = " << pose_real.position.y << endl; 
        cout << "iRobot.x = " << nmpc.iRobot.x_rob << ", iRobot.y = " << nmpc.iRobot.y_rob << endl;
        cout << "amcl.x = " << nmpc.cloud.x << ", amcl.y = " << nmpc.cloud.y  << endl;
        cout << "velo.linear.x = " << msg.linear.x << ", velo.angular.z = " << msg.angular.z << endl;
        
        velPub.publish(msg);

        loop_rate1.sleep(); 
                

        cout << endl;
    }
    


    

	return 0;
}

