/*
Best performance so far for the xyz_controller
*/

#ifndef LQ_SERVO_H
#define LQ_SERVO_H

#include "ros/ros.h"
#include <math.h>
#include <cmath>
#include <vector>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/Actuators.h>
#include <sensor_msgs/Imu.h>
#include <ORTILo/StateVector.h>
#include <ORTILo/ControlVector.h>
#include <Eigen/Core>

namespace lq_servo
{

class LQServoController
{
    public:
        // Constructor
        LQServoController();

        // Callback functions
        void estimatedStateCallback(const ORTILo::StateVectorConstPtr &msg);
        void commandedPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);

        void outputCallback(const geometry_msgs::PoseStampedConstPtr &msg);
        void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
        void singlePoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
        
        // LQI related functions
        void lqi();

        // Math related functions        
        void matrice_mul(std::vector<std::vector<double>> &A, std::vector<std::vector<double>> &B, std::vector<std::vector<double>> &result, uint8_t row_a, uint8_t column_a, uint8_t row_b, uint8_t column_b);
        void motorSpeeds(std::vector<std::vector<double>> &control_vector_);
        void print_mat(std::vector<std::vector<double>> &mat, uint8_t rows, uint8_t columns);
        std::vector<std::vector<double>> quatToEuler(double x, double y, double z, double w);

    private:

        ros::NodeHandle nh;
        ros::NodeHandle nh_private;

        ros::Subscriber estimated_state_sub; // estimated state from the Kalman Filter
        ros::Subscriber commanded_pose_sub;

        ros::Subscriber output_sub; //y = C*X
        ros::Subscriber velocity_sub;
        ros::Subscriber single_pose_sub;

        ros::Publisher command_pub;
        ros::Publisher motor_speeds_pub;
    
        geometry_msgs::PoseStamped commanded_pose;
        geometry_msgs::PoseStamped plant_output;
        ORTILo::StateVector estimated_state;
        geometry_msgs::PoseStamped single_desired_position;
        mav_msgs::Actuators motor_speeds;
        ORTILo::ControlVector control_vector;

		static const uint8_t row_L = 4;
		static const uint8_t row_X = 12;
		static const uint8_t column_X = 1;
        static const uint8_t row_r = 4;
        static const uint8_t column_r = 1;

		static const uint8_t row_E = 4;
		static const uint8_t column_E = 6;
        static const uint8_t row_Y = 6;
		static const uint8_t column_Y = 1;
		static const uint8_t row_integral_result = 3;
		static const uint8_t column_integral_result = 1;
		static const uint8_t column_X_aug = 1;
        static const uint8_t row_U = 4;
        static const uint8_t column_U = 1;
        const float max_rot_velocity = 620.0; // rad/s
        const float min_rot_velocity = 550.0; // rad/s        
        const float hover_rot_velocity = 595.0; // rad/s

        int counter = 0;
        float s = 0.7071067812; // sin(45)

        std::vector<std::vector<double>> eulerAngles
        {
            {0.0}, // roll
            {0.0}, // pitch
            {0.0}  // yaw
        };

        std::vector<std::vector<double>> X
        {
            {0.0},
            {0.0},
            {0.0},
            {0.0},
            {0.0},
            {0.0},
            {0.0},
            {0.0},
            {0.0},
            {0.0},
            {0.0},
            {0.0}                                                                       
        };        

        std::vector<std::vector<double>> U
        {
            {0.0},
            {0.0},
            {0.0},
            {0.0}                        
        };

        double Xi = 0.0;
        double Yi = 0.0;
        double Zi = 0.0;
        double yawi = 0.0;

        std::vector<std::vector<double>> X_aug
        {
            {0.0},
            {0.0},
            {0.0},
            {0.0},
            {0.0},
            {0.0},
            {0.0},
            {0.0},
            {0.0},
            {0.0},
            {0.0},
            {0.0},
            {0.0},
            {0.0},
            {0.0},
            {0.0}
        };         
		static const uint8_t row_X_aug = row_X+4;
        static const uint8_t column_L = 16;
   

        std::vector<std::vector<double>> L
        {
            {0.000000, 0.000000, 6.832947, -0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 8.006557, -0.000000, 0.000000, 0.000000, 0.000000, -0.000000, -0.095022, -0.000000},
            {0.000000, -2.061833, -0.000000, 5.964862, 0.000000, -0.000000, 0.000000, -1.603052, -0.000000, 1.330474, 0.000000, 0.000000, -0.000000, 0.001102, 0.000000, -0.000000},
            {1.802113, 0.000000, -0.000000, -0.000000, 5.770303, -0.000000, 1.472012, 0.000000, 0.000000, 0.000000, 1.356212, 0.000000, -0.000963, 0.000000, -0.000000, 0.000000},
            {0.000000, -0.000000, -0.000000, 0.000000, 0.000000, 2.843224, 0.000000, -0.000000, 0.000000, 0.000000, 0.000000, 1.400933, -0.000000, -0.000000, -0.000000, -0.08847}
        };              

        //[3][6]
        std::vector<std::vector<double>> E
        {
			{1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
			{0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
			{0.0, 0.0, 1.0, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0, 0.0, 0.0, 1.0}
        };

        std::vector<std::vector<double>> Y // These values are the initial pose of the Quadrotor
        {
			{0.0},
			{0.0},
			{0.08},
			{0.0},
			{0.0},
			{0.0}
        };
        std::vector<std::vector<double>> E_times_Y
        {
			{0.0},
			{0.0},
			{0.0},
            {0.0}
        };        
        std::vector<std::vector<double>> r
        {
			{0.0},
			{0.0},
			{0.08},
            {0.0}
        };
    

        int flag = 0;
        int count_m0 = 0;
        int count_m1 = 0;
        int count_m2 = 0;
        int count_m3 = 0;
};

}

#endif
