#include "../include/nmpc.h"
#include <math.h>

Vout NMPC::NMPController(MyRobot iRobot, Trajectory Traj){

    

    TRobotStateSim simRobot;
    TTargetStateSim simTarget;

    Matrix<double,8,1> JSteps;
    Matrix<double,4,1> Jgradient;
    Matrix<double,4,1> Jgradient_prev;
    Matrix<double,4,1> Jsteps_prev;
    int iterationCount;

    JSteps.fill(0);
    Jgradient.fill(0);
    Jgradient_prev.fill(0);
    Jsteps_prev.fill(0.1);
    iterationCount = 0;

    Matrix2d Uaux;
    Matrix2d Uref;
    Matrix2d Ubest;
    Matrix<double,2,8> Usteps;

    Uaux.fill(0);
    Uref.fill(0);
    Ubest.fill(0);
    Usteps.fill(0); 

    double Jcurrent,Jbest,J;
    double currStep, currGrad, prevStep, prevGrad;

    simRobot.x = 0;
    simRobot.y = 0;
    simRobot.teta = 0;
    simRobot.v = 0;
    simRobot.w = 0;

    simTarget.x = 0;
    simTarget.y = 0;
    simTarget.vx = 0;
    simTarget.vy = 0;

    for(int i=0;i< NU3;i++){
    //    Uref.setv(0,i,iRobot.v_rob);    //v
    //    Uref.setv(1,i,iRobot.w_rob);    //w
        Uref(0,i) = Traj.v_ref;    //v
        Uref(1,i) = Traj.w_ref;    //w
    }

    simRobot.x = iRobot.x_rob;
    simRobot.y = iRobot.y_rob;
    simRobot.teta = iRobot.teta_rob;
    simRobot.v = iRobot.v_rob;
    simRobot.w = iRobot.w_rob;

    simTarget.x = Traj.x_ref;
    simTarget.y = Traj.y_ref;
    simTarget.vx = Traj.vx_ref;
    simTarget.vy = Traj.vy_ref;

    // cout << "Parametros = iRobot.x_out = " << iRobot.x_rob << endl
    //     << "\tiRobot.y_out = " << iRobot.y_rob << endl
    //     << "\tiRobot.teta_out = " << iRobot.teta_rob << endl
    //     << "\tiRobot.v_out = " << iRobot.v_rob << endl
    //     << "\tiRobot.w_out = " << iRobot.w_rob << endl
    //     << "\tTraj.x_ref = " << Traj.x_ref << endl
    //     << "\tTraj.y_ref = " << Traj.y_ref << endl
    //     << "\tTraj.vx_ref = " << Traj.vx_ref << endl
    //     << "\tTraj.vy_ref = " << Traj.vy_ref << endl;

    Uref = saturate(Uref);
    // cout << "Linha 75, Uref = " << Uref << endl;
    
    Jcurrent = CostFunction(simRobot, simTarget, Uref);

    Jbest = Jcurrent;
    // cout << "Linha 81, Jcurrent e Jbest = " << Jcurrent << endl;

    //---------------------------------------------------
    //              Optimization loop
    //---------------------------------------------------
    while ((iterationCount<IMAX)&&(Jcurrent>JSTOP)){
        
        //get Usteps matrix
        Usteps = calcUSteps(Uref);
        // cout << "Linhas 89, Usteps = " << Usteps << endl;

        //Calculate Jsteps vector (do one simulation for each input set)
        for (int k=0;k<NU3;k++){
            for (int ib=0; ib<4;ib++){
                for (int mb=0;mb<NU3;mb++){
                    if (mb==k){
                        Uaux(0,mb) = Usteps(0,ib+4*k);
                        Uaux(1,mb) = Usteps(1,ib+4*k);
                    }
                    else{
                        Uaux(0,mb) = Uref(0,0);
                        Uaux(1,mb) = Uref(1,0);
                    }
                }

                //Reset robot initial state for each simulation
                simRobot.x = iRobot.x_rob;
                simRobot.y = iRobot.y_rob;
                simRobot.teta = iRobot.teta_rob;
                simRobot.v = iRobot.v_rob;
                simRobot.w = iRobot.w_rob;

                simTarget.x = Traj.x_ref;
                simTarget.y = Traj.y_ref;
                simTarget.vx = Traj.vx_ref;
                simTarget.vy = Traj.vy_ref;

                //Limit wheel speed references in case of motor saturation (update references)
                Uaux = saturate(Uaux);
                // cout << "Linha 119, Uaux = " << Uaux << endl;
                    
                //Do simulation with current Uaux and add to Jsteps vector
                //Switches between trajectory controller and formation controller
                J = CostFunction(simRobot, simTarget, Uaux);
                // cout << "Linhas 124, J = " << J << endl;

                //Add J to Jsteps
                JSteps(ib+4*k,0) = J;
                // cout << "Linhas 128, Jsteps = " << JSteps << endl;
            }
        }     

        //Compute gradient of J from Jsteps
        for (int i=0;i<NU3;i++){
            Jgradient_prev(2*i+0,0) = Jgradient(2*i+0,0);
            Jgradient(2*i+0,0) = JSteps(4*i+0,0)-JSteps(4*i+1,0);

            Jgradient_prev(2*i+1,0) = Jgradient(2*i+1,0);
            Jgradient(2*i+1,0) = JSteps(4*i+2,0)-JSteps(4*i+3,0);
        }
        // cout << "Linha 138, Jgradient_prev = " << Jgradient_prev << endl;
        // cout << "Linha 139, Jgradient = " << Jgradient << endl;

        //Minimization algorithm
        for (int i=0; i<NU3; i++){
            for (int j=0; j<=1; j++){
                prevStep = Jsteps_prev(2*i+j,0);
                currGrad = Jgradient(2*i+j,0);

                if (prevGrad*currGrad > 0){
                    currStep = min(prevStep*ETA_P, STEP_MAX);
                    Uref(j,i) = (Uref(j,i)-(mysign(currGrad)*currStep));
                    prevGrad = currGrad;
                }
                else{
                    if (prevGrad*currGrad < 0){
                        currStep = max(prevStep*ETA_M, STEP_MIN);
                        prevGrad = 0;
                    }
                    
                    if (prevGrad*currGrad == 0){
                        Uref(j,i) = (Uref(j,i)-(mysign(currGrad)*prevStep));
                        prevGrad = currGrad;
                    }
                    Jsteps_prev(2*i+j,0) = currStep;
                }
            }
        }
        // cout << "Linha 148, currGrad = " << currGrad << endl;
        // cout << "Linha 157, currStep = " << currStep << endl;
        // cout << "Linha 162, Uref = " << Uref << endl;
        // cout << "Linha 163, prevGrad = " << prevGrad << endl;
        // cout << "Linha 165, Jsteps_prev = " << Jsteps_prev << endl;
        

        simRobot.x = iRobot.x_rob;
        simRobot.y = iRobot.y_rob;
        simRobot.teta = iRobot.teta_rob;
        simRobot.v = iRobot.v_rob;
        simRobot.w = iRobot.w_rob;

        simTarget.x = Traj.x_ref;
        simTarget.y = Traj.y_ref;
        simTarget.vx = Traj.vx_ref;
        simTarget.vy = Traj.vy_ref;

        Uref = saturate(Uref);
        // cout << "Linha 187, Uref = " << Uref << endl;

        //Calculate new current cost (do simulation)
        Jcurrent = CostFunction(simRobot, simTarget, Uref);
        // cout << "Linha 191, Jcurrent = " << Jcurrent << endl;

        //Update JBest
        if (Jcurrent < Jbest){
            Jbest = Jcurrent;
            Ubest(0,0) = Uref(0,0);
            Ubest(1,0) = Uref(1,0);
        }
        // cout << "Linha 198, Ubest = " << Ubest << endl;

        iterationCount += 1;

    }

    velocity.v_out = Ubest(0,0);
    velocity.w_out = Ubest(1,0);

    //cout << "v: " << velocity.v_out << ", w: " << velocity.w_out << "\n";

    return velocity;
}

// void NMPC::OdomCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){

//     //subscreve valores do EKF
//     iRobot.x_rob =  msg->pose.pose.position.x;
//     iRobot.y_rob =  msg->pose.pose.position.y;
//     iRobot.teta_rob = msg->pose.pose.orientation.z;
// }


void NMPC::OdomCallback(const nav_msgs::Odometry::ConstPtr& vel){

    //subscreve valores de velocidade
    iRobot.v_rob = vel->twist.twist.linear.x;
    iRobot.w_rob = vel->twist.twist.angular.z; 
    iRobot.x_rob = vel->pose.pose.position.x;
    iRobot.y_rob = vel->pose.pose.position.y;
    iRobot.teta_rob = 2*atan2(vel->pose.pose.orientation.z,
                              vel->pose.pose.orientation.w);
    

}

void NMPC::CloudCallback(const geometry_msgs::PoseArray& pose){

    int size = pose.poses.size();
    
    for(int i = 0; i < size; i++){
        cloud.x += pose.poses[i].position.x;
        cloud.y += pose.poses[i].position.y;        
    }
    
    cloud.x = cloud.x/size;
    cloud.y = cloud.y/size;
    
    

}

double NMPC::CostFunction(TRobotStateSim Robot, TTargetStateSim Target, Matrix2d& Ut){

    double sum_cost;
    double res, deltaU;
    double v, w, cteta, steta;
    double RobotTargetDist, RB_x, RB_y, RobotTargetAngle;
    
    sum_cost = 0;
  

    for(int i=NU1; i<=NU2; i++){

        if (i<=NU3){
            v = Ut(0,i-1);
            w = Ut(1,i-1);
        }
        else{
            v = Ut(0,NU3-1);
            w = Ut(1,NU3-1);
        }
    
        for(int j=0;j<=3;j++){
            cteta = cos(Robot.teta);
            steta = sin(Robot.teta);

            if (Robot.teta > M_PI){
                Robot.teta = Robot.teta - 2*M_PI; 
            }

            Robot.teta = Robot.teta + w*SIM_TIME_STEP;

            Robot.x = Robot.x + SIM_TIME_STEP*(v*cteta);
            Robot.y = Robot.y + SIM_TIME_STEP*(v*steta);

            Target.x = Target.x + SIM_TIME_STEP*Target.vx;
            Target.y = Target.y + SIM_TIME_STEP*Target.vy;

            Target.vx = Target.vx*BFC;
            Target.vy = Target.vy*BFC;
        }

        RobotTargetDist = sqrt(pow((Target.x - Robot.x),2)+pow((Target.y - Robot.y),2));

        RB_x = (Target.x - Robot.x)/RobotTargetDist;
        RB_y = (Target.y - Robot.y)/RobotTargetDist;

        RobotTargetAngle = atan2(RB_y,RB_x);

        sum_cost = sum_cost + Le1*fabs(DVAL - RobotTargetDist);

        sum_cost = sum_cost + Le2*fabs(DiffAngle(RobotTargetAngle,Robot.teta));
    }

    deltaU = Le3*(fabs(Robot.v-Ut(0,0))+fabs(Robot.w-Ut(1,0)));
    // cout << "deltaU = " << deltaU << endl;
    res = (sum_cost + deltaU);

    return res;
}

double NMPC::DiffAngle(double a1, double a2){
    int ang;

    ang = a1-a2;
    // cout << ang << endl;

    if (ang<0){
        ang = -((-ang/(2*M_PI))-floor((-ang/(2*M_PI)))*2*M_PI);
    }

    if (ang<M_PI){
        ang = ang + (2*M_PI);
        // cout << ang << endl;
    }
    else {
        ang = ((ang/(2*M_PI))-floor((ang/(2*M_PI))))*(2*M_PI);
    }

    if (ang>M_PI){
        ang = ang - (2*M_PI);
        // cout << ang << endl;
    }

    return ang;
}

Matrix2d NMPC::saturate(Matrix2d& u){
    double v, w, v1, v2;
    double maxv, minv, scaleMax, scaleMin, scale;
  
    Matrix2d ut;
    ut.fill(0);

    for(int i=0; i<NU3; i++){
        v=u(0,i);
        w=u(1,i);

        v1 = v + ((d_Rob*w)/2);
        v2 = v - ((d_Rob*w)/2);

        maxv = max(v1,v2);
        minv = min(v1,v2);

        if (maxv>VMAX){
            scaleMax=maxv/VMAX;
        }
        else {
            scaleMax=1;
        }

        if (minv<(-VMAX)){
            scaleMin=minv/(-VMAX);
        }
        else{
            scaleMin=1;
        }

        scale = max(scaleMax, scaleMin);

        v1 = v1/scale;
        v2 = v2/scale;

        ut(0,i) = (v1+v2)/2;
        ut(1,i) = (v1-v2)/d_Rob;

        
    }

     return ut;
}

Matrix<double,2,8> NMPC::calcUSteps(Matrix2d& Ud){

    Matrix<double,2,8> deltaU;
    deltaU.fill(0);

    for (int i=0;i< NU3;i++){
      //col 1 (v+d,vn,w)
      deltaU(0,0+4*i) = Ud(0,i)+DELTA;
      deltaU(1,0+4*i) = Ud(1,i);
      //col 2 (v-d,vn,w)
      deltaU(0,1+4*i) = Ud(0,i)-DELTA;
      deltaU(1,1+4*i) = Ud(1,i);
      //col 3 (v,vn+EditDelta,w)
      deltaU(0,2+4*i) = Ud(0,i);
      deltaU(1,2+4*i) = Ud(1,i)+DELTA;
      //col 4 ...
      deltaU(0,3+4*i) = Ud(0,i);
      deltaU(1,3+4*i) = Ud(1,i)-DELTA;
    }
    return deltaU;
}

int NMPC::mysign(double x){
    if (x > 0.0) return 1;
    if (x < 0.0) return -1;
    
    return 0;
}


