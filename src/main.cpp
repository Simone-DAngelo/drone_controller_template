#include "rclcpp/rclcpp.hpp"
#include "controller/controller.h"

#include "boost/thread.hpp"
#include <Eigen/Eigen>

// PX4 Libraries
#include "std_msgs/msg/float32_multi_array.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_thrust_setpoint.hpp"
#include "px4_msgs/msg/vehicle_torque_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"

// Custom Libraries
#include "utils.h"

#include <cstdio>
#include <chrono>

using namespace std::chrono_literals;
using namespace std;
using namespace Eigen;

using std::placeholders::_1;

class CONTROLLER : public rclcpp::Node {
    public:
        CONTROLLER();
        void run();
        void ctrl_loop();
        void request_new_plan();
        bool get_allocation_matrix(Eigen::MatrixXd & allocation_M, int motor_size );
        void traj_comput();
        void arm();
        void disarm();

    private:
        void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
        void publish_thrust_setpoint(float thrust);
        void publish_torque_setpoint(Eigen::Vector3d torque);
        void publish_offboard_control_mode();
        void timerCallback();

        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _odom_sub;
        
        rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr _vehicle_thrust_sp_publisher; 
        rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr _vehicle_torque_sp_publisher; 
        rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr _offboard_control_mode_publisher;
	    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _vehicle_command_publisher;

        bool _first_odom, _new_plan;

        //---Parameters
        string _model_name;
        double _ctrl_rate;
        int _motor_num;
        Eigen::Matrix3d _inertia;
        Eigen::Vector3d _position_gain;
        Eigen::Vector3d _velocity_gain;
        Eigen::Vector3d _attitude_gain;
        Eigen::Vector3d _angular_rate_gain;
        Eigen::VectorXd _omega_motor;
        double _mass;
        double _gravity;
        vector<double> _rotor_angles;
        vector<double> _arm_length;
        double _motor_force_k;
        double _motor_moment_k;
        vector<int> _motor_rotation_direction;
        
        Eigen::Vector3d _perror;
        Eigen::Vector3d _verror;

        Vector3d _ref_p;
        Vector3d _ref_dp;
        Vector3d _ref_ddp;
        double _ref_yaw;
        double _traj_rate;
        Vector3d _cmd_p;
        Vector3d _cmd_dp;
        Vector3d _cmd_ddp;
        
        Vector4d _att_q;
        double _yaw_cmd;
        double _ref_dyaw;
        double _ref_ddyaw;
        Eigen::Vector3d _mes_p;
        Eigen::Vector3d _mes_dp;
        Eigen::Vector3d _omega_mes;

        float _thrust_normalized;
        Eigen::Vector3d _torque_normalized;
        Eigen::Vector4d _max_wrench, _max_rot_speed;
        bool _armed;
};

CONTROLLER::CONTROLLER() : Node("drone_control"), _first_odom(false), _new_plan(false) {
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&CONTROLLER::timerCallback, this));

    // Get param from file yaml --------------------------------------- 
    declare_parameter("model_name","uav");
    // TODO: declare all parameters with default values

    auto _mod_name = get_parameter("model_name").as_string();
    // TODO: get all parameters from yaml file

    _model_name = _mod_name;
    // set parameters to class variables

    // ROS2 pub/sub
    // Set proper QoS profile for subscriptions and publications
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    
    // Subscribers
    _odom_sub = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos, 
    [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) -> void {
        _mes_p << msg->position[0], msg->position[1], msg->position[2];
        _mes_dp << msg->velocity[0], msg->velocity[1], msg->velocity[2];
        _att_q << msg->q[0], msg->q[1], msg->q[2], msg->q[3];
        _omega_mes << msg->angular_velocity[0], msg->angular_velocity[1], msg->angular_velocity[2];
        _first_odom = true;
    });

    // Publishers
    _vehicle_thrust_sp_publisher = this->create_publisher<px4_msgs::msg::VehicleThrustSetpoint>("/fmu/in/vehicle_thrust_setpoint", 0);
    _vehicle_torque_sp_publisher = this->create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>("/fmu/in/vehicle_torque_setpoint", 0);
    _vehicle_command_publisher = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 0);
    _offboard_control_mode_publisher = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 0);

    // Set class variables to default values
    _cmd_p << 0.0, 0.0, 0.0;
    _cmd_dp << 0.0, 0.0, 0.0;
    _cmd_ddp << 0.0, 0.0, 0.0;
    _ref_yaw = 0.0;
    _omega_motor.resize( _motor_num );
    for(int i=0; i<_motor_num; i++ )
      _omega_motor[i] = 0.0; 

    _thrust_normalized = 0.0;
    _torque_normalized << 0,0,0;  
    _max_rot_speed << pow(1100,2), pow(1100,2), pow(1100,2), pow(1100,2);
    _armed = false;
}

void CONTROLLER::timerCallback() {
    if(_first_odom){
        std_msgs::msg::Float32MultiArray motor_vel;
        motor_vel.data.resize( _motor_num );
    
        if(_cmd_p[2]<=-0.2 && !_armed){
            publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            arm();
            _armed = true;
        }
        
        if(_new_plan){
            publish_offboard_control_mode();
            publish_thrust_setpoint(_thrust_normalized); 
            publish_torque_setpoint(_torque_normalized);
        }
        else{
            publish_offboard_control_mode();
            publish_thrust_setpoint(0.1); 
            publish_torque_setpoint(Eigen::Vector3d(0.0, 0.0, 0.0));
        }
        
    }
    else{
        RCLCPP_INFO(this->get_logger(), "WAITING FOR ODOM...");
    }
}

// PX4 FUNCTIONS ENABLING OFFBOARD CONTROL

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void CONTROLLER::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	px4_msgs::msg::VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	_vehicle_command_publisher->publish(msg);
}

/**
 * @brief Send a command to Arm the vehicle
 */
void CONTROLLER::arm(){
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void CONTROLLER::disarm(){
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void CONTROLLER::publish_thrust_setpoint(float thrust){
	px4_msgs::msg::VehicleThrustSetpoint msg{};
    
	msg.xyz[0] = 0;
    msg.xyz[1] = 0;
    msg.xyz[2] = thrust;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	_vehicle_thrust_sp_publisher->publish(msg);
}

void CONTROLLER::publish_torque_setpoint(Eigen::Vector3d torque)
{
	px4_msgs::msg::VehicleTorqueSetpoint msg{};
    
	msg.xyz[0] = torque[0];
    msg.xyz[1] = torque[1];
    msg.xyz[2] = torque[2];
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	_vehicle_torque_sp_publisher->publish(msg);
}

// PX4 function specializing offboard mode to allow actuator control (thrust and torque setpoints)
// From 1.15 is possible to control directly the actuators (motor velocities) check user guide for more information
// https://docs.px4.io/main/en/msg_docs/OffboardControlMode.html
void CONTROLLER::publish_offboard_control_mode()
{
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = false;
    msg.velocity = false;
    msg.acceleration = false;
    msg.body_rate = false;
    msg.attitude = false;
    msg.actuator = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    _offboard_control_mode_publisher->publish(msg);
}

// Send a new set point to the trajectory planner
void CONTROLLER::request_new_plan() {

    // TODO CHANGE THE LOGIC AS YOU NEED

    float set_x, set_y, set_z, set_yaw;
    while(rclcpp::ok()) {
        cout << "Insert new coordinates x (front), y (right), z (downword), yaw (clowise)" <<endl;
        scanf("%f %f %f %f", &set_x, &set_y, &set_z, &set_yaw);
        cout << "Request new plan for: [" << set_x << ", " << set_y << ", " << set_z << " - " << set_yaw << "]" << endl;

        //---ENU -> NED
        _cmd_p << set_x, set_y, set_z;
        _yaw_cmd = set_yaw;
        //---
        _new_plan = true;
    }
    _new_plan = false;
}  

// Function to generate the allocation matrix for the quadrotor
bool generate_allocation_matrix(Eigen::MatrixXd & allocation_M, 
                                    int motor_size,
                                    vector<double> rotor_angle,
                                    vector<double> arm_length, 
                                    double force_k,
                                    double moment_k,
                                    vector<int> direction ) {

    allocation_M.resize(4, motor_size );

    // TODO: set allocation matrix and check on correctness, return true if rank is 4 (for quadrotor)

    return true;
}

// Function to compute the 4-dofs desired trajectory
void CONTROLLER::traj_comput(){
  
   // TODO: implement trajectory planning logic

    while( !_first_odom ) usleep(0.1*1e6);

    rclcpp::Rate r(_traj_rate);
    
            
    while( rclcpp::ok() ) {
        

        r.sleep();
    }
}

// CONTROL LOOP
void CONTROLLER::ctrl_loop() {

    rclcpp::Rate r(_ctrl_rate);

    //---Input
    Eigen::Vector3d des_p;              
    Eigen::Vector3d des_dp; 
    Eigen::Vector3d des_ddp; 
    des_dp << 0.0, 0.0, 0.0;
    des_ddp << 0.0, 0.0, 0.0;

    Eigen::Vector4d mes_q;
    Eigen::Vector3d mes_w;

    //---

    Eigen::MatrixXd allocation_M;
    Eigen::MatrixXd wd2rpm;
    
    while( !_first_odom ) usleep(0.1*1e6);
    
    if(!generate_allocation_matrix( allocation_M, _motor_num, _rotor_angles, _arm_length, _motor_force_k, _motor_moment_k, _motor_rotation_direction ) ) {     
        cout << "Wrong allocation matrix" << endl;
        exit(0);
    }

    // ----- max thrust and torque computation
        // TODO compute maximum thrust and torque based on the allocation matrix to normalize the commands
        _max_wrench << 0.0, 0.0, 0.0, 0.0;
        cout<<"Vector of maximum command wrench: "<<_max_wrench.transpose()<<endl;
    // ---------------------------------------

    boost::thread input_t( &CONTROLLER::request_new_plan, this);
    boost::thread traj_comput_t(&CONTROLLER::traj_comput, this);

    wd2rpm.resize( _motor_num, 4 );
    Eigen::Matrix4d I;
    I.setZero();
    I.block<3, 3>(0, 0) = _inertia;
    I(3, 3) = 1;
    
    CONTROLLER lc;
    lc.set_uav_dynamics( _motor_num, _mass, _gravity, I);
    lc.set_controller_gains( _position_gain, _velocity_gain, _attitude_gain, _angular_rate_gain );
    lc.set_allocation_matrix( allocation_M );
     
    Eigen::VectorXd ref_rotor_velocities;
    Eigen::Vector4d ft;
    
    Vector3d att_err;

    while( rclcpp::ok() ) {
        //Measured    
        mes_q = _att_q;
        mes_w = _omega_mes;          
        Eigen::Matrix3d mes_R = utilities::QuatToMat( mes_q );
    
        lc.controller_fun(_mes_p, _ref_p, mes_R, _mes_dp, _ref_dp, _ref_ddp, _ref_yaw, _ref_dyaw, _ref_ddyaw, mes_w, &ref_rotor_velocities, &ft, &_perror, &_verror, &att_err);   
        
        // SATURATION AND COMMAND NORMALIZATION FOR PX4
        if(ft[3]<-_max_wrench(3)){
            ft[3]=-_max_wrench(3);
        }
        
        _thrust_normalized = ft(3) / _max_wrench(3);
        _thrust_normalized = std::clamp(_thrust_normalized, -1.0f, 0.0f);

        _torque_normalized(0) = ft(0) / _max_wrench(0);
        _torque_normalized(1) = ft(1) / _max_wrench(1);
        _torque_normalized(2) = ft(2) / _max_wrench(2);

        for(int i = 0; i < 3; i++) {
            _torque_normalized(i) = std::clamp(_torque_normalized(i), -1.0, 1.0);
        }

        for(int i=0; i<_motor_num; i++ ) {
            _omega_motor[i] = ref_rotor_velocities[i]; 
        }

        // IF you want to control the motors directly, you have to normalize directly the param ref_rotor_velocities. 
        // Knowing the maximum speed of the motors, you can easily compute them as signals (PWM) between 0 and 1000 
        // If you use gazebo classic then sum 1000 before publish. 
        // If you use gazebo ignition, publish directly the normalized value between 0 and 1000 on the PX4 topic actuator_motors.
        // Do not forget to properly set the function offboard_control_mode() to allow actuator control.
        // https://docs.px4.io/main/en/msg_docs/OffboardControlMode.html
        r.sleep();
    }   
}

void CONTROLLER::run() {
    boost::thread ctrl_loop_t( &CONTROLLER::ctrl_loop, this );  
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CONTROLLER>();
    node->run();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}