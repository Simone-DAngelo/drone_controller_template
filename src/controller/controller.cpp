#include "controller.h"

using namespace std;

CONTROLLER::CONTROLLER() {}

void CONTROLLER::set_allocation_matrix(  Eigen::MatrixXd allocation_M ) {
  _wd2rpm = allocation_M.transpose() * (allocation_M*allocation_M.transpose()).inverse();   
}

void CONTROLLER::set_uav_dynamics (int motor_num, double mass, double gravity, Eigen::Matrix4d I) {
  _mass = mass;
  _gravity = gravity;
  _I = I;
  _motor_num = motor_num;
}

void CONTROLLER::set_controller_gains(Eigen::Vector3d kp, Eigen::Vector3d kd, Eigen::Vector3d attitude_gain, Eigen::Vector3d angular_rate_gain ) {
  _kp = kp;
  _kd = kd;
  _attitude_gain = attitude_gain;
  _angular_rate_gain = angular_rate_gain;
}

void CONTROLLER::controller_fun(  Eigen::Vector3d mes_p, 
                                  Eigen::Vector3d des_p,  
                                  Eigen::Matrix3d mes_R,
                                  Eigen::Vector3d mes_dp, 
                                  Eigen::Vector3d des_dp,    
                                  Eigen::Vector3d des_ddp,
                                  double des_yaw,
                                  double des_dyaw,
                                  double des_ddyaw,
                                  Eigen::Vector3d mes_w,
                                  Eigen::VectorXd* rotor_velocities,
                                  Eigen::Vector4d* ft,
                                  Eigen::Vector3d* perror,
                                  Eigen::Vector3d* verror,
                                  Eigen::Vector3d* att_error ) {

                                      
    // YOUR CONTROLLER CODE HERE


    // *rotor_velocities = _wd2rpm * angular_acceleration_thrust;
    // *ft = angular_acceleration_thrust;
    // *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
    // *rotor_velocities = rotor_velocities->cwiseSqrt();

    // *perror = position_error;
    // *verror = velocity_error;
      
}
