#include "nav_msgs/msg/odometry.hpp"
#include "boost/thread.hpp"
#include <Eigen/Eigen>

class CONTROLLER {

    public:
        CONTROLLER();
        void controller_fun(
                        Eigen::Vector3d mes_p, 
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
                        Eigen::Vector3d* att_error );

        void set_allocation_matrix( Eigen::MatrixXd allocation_M );
        void set_controller_gains(Eigen::Vector3d kp, Eigen::Vector3d kd, Eigen::Vector3d attitude_gain, Eigen::Vector3d angular_rate_gain );
        void set_uav_dynamics (int _motor_num, double mass, double gravity, Eigen::Matrix4d I);
        
        // FEEL FREE TO ADD MORE FUNCTIONS IF NEEDED
        // EX: Why not decouple attitude control from position control?

    private:
        // ADAPT DIMENSIONS AND TYPES AS NEEDED TEMPLATE 
        // WORKING WITH STANDARD QUADROTOR

        Eigen::Vector3d _kp;
        Eigen::Vector3d _kd;
        Eigen::Vector3d _attitude_gain;
        Eigen::Vector3d _angular_rate_gain;
        Eigen::MatrixXd _wd2rpm;
        Eigen::Matrix4d _I;
        double _mass;
        double _gravity;
        int _motor_num;
};