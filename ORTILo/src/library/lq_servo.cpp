/*
Best performance so far for the xyz controller
*/

#include <ORTILo/lq_servo.h>

namespace lq_servo
{
	LQServoController::LQServoController() :
    	nh(ros::NodeHandle()),
    	nh_private("~")
	{
  		// Set up Publishers and Subscribers
		printf("Initializing Subscribers and Publishers ...\n");
  		estimated_state_sub = nh.subscribe("/kalman_filter_estimated_state", 1, &LQServoController::estimatedStateCallback, this); // 'X' and 'Y'
		commanded_pose_sub = nh.subscribe("/commanded_pose", 1, &LQServoController::commandedPoseCallback, this); // 'r'
		
		output_sub = nh.subscribe("/uav1/pose", 1, &LQServoController::outputCallback, this); // 'Y', 'X:pose'
		//velocity_sub = nh.subscribe("/uav1/imu", 1, &LQServoController::imuCallback, this); // 'X:angular velocities'
		velocity_sub = nh.subscribe("/uav1/odometry", 1, &LQServoController::odometryCallback, this); // 'X:velocities'
		single_pose_sub = nh.subscribe("/lqi/command/single_pose", 1, &LQServoController::singlePoseCallback, this); // 'r'

		command_pub = nh.advertise<ORTILo::ControlVector>("/control_input", 1); // 'U:[thrust, tau_x, tau_y, tau_z]'
		motor_speeds_pub = nh.advertise<mav_msgs::Actuators>("/uav1/command/motor_speed", 1); // 'U*:[m0, m1, m2, m3, m4]'
		
		// Set size of the motor_speeds vector
		motor_speeds.angular_velocities.push_back(hover_rot_velocity);
		motor_speeds.angular_velocities.push_back(hover_rot_velocity);
		motor_speeds.angular_velocities.push_back(hover_rot_velocity);
		motor_speeds.angular_velocities.push_back(hover_rot_velocity);		

		printf("All publishers and subscribers have been succesfully initialized\n");	
	}

	void LQServoController::estimatedStateCallback(const ORTILo::StateVectorConstPtr &msg)
	{
		estimated_state.x = msg->x;
		estimated_state.y = msg->y;
		estimated_state.z = msg->z;
		estimated_state.phi = msg->phi;
		estimated_state.theta = msg->theta;
		estimated_state.psi = msg->psi;
		
	}
	void LQServoController::commandedPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
	{
		commanded_pose.pose = msg->pose;
	}

	void LQServoController::outputCallback(const geometry_msgs::PoseStampedConstPtr &msg)
	{
		plant_output.pose = msg->pose;
	}

	void LQServoController::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
	{
		estimated_state.u_x = msg->twist.twist.linear.x;
		estimated_state.u_y = msg->twist.twist.linear.y;
		estimated_state.u_z = msg->twist.twist.linear.z;
		estimated_state.w_phi = msg->twist.twist.angular.x;
		estimated_state.w_theta = msg->twist.twist.angular.y;
		estimated_state.w_psi = msg->twist.twist.angular.z;
	}

	void LQServoController::singlePoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
	{
		single_desired_position.pose.position = msg->pose.position;
		r.at(0).at(0) = single_desired_position.pose.position.x;
		r.at(1).at(0) = single_desired_position.pose.position.y;
		r.at(2).at(0) = single_desired_position.pose.position.z;		
	}

	std::vector<std::vector<double>> LQServoController::quatToEuler(double x, double y, double z, double w)
	{
		// Quaternions(x, y, z, w) to Euler Angles (roll, pitch, yaw)
		// roll: 	rotation in radians about the X-axis in a counterclockwise direction
		// pitch:	rotation in radians about the Y-axis in a counterclockwise direction
		// yaw: 	rotation in radians about the Z-axis in a counterclockwise direction

		float temp0, temp1, temp2, temp3, temp4;

        temp0 = 2.0*(x*w + y*z);
        temp1 = 1.0 - 2.0*(x*x + y*y);
        float roll = atan2(temp0, temp1);
     
        temp2 = 2.0*(y*w - x*z);
        if(temp2>1.0)
			temp2 = 1.0;
		if(temp2<-1.0)
			temp2 = -1.0;
        float pitch = asin(temp2);
     
        temp3 = 2.0*(x*y + z*w);
        temp4 = 1.0 - 2.0*(y*y + z*z);
        float yaw = atan2(temp3, temp4);

		/*
		std::cout << "\n------------ EULER ANGLES ------------\n";
		std::cout <<"roll = " << roll << "\n";
		std::cout <<"pitch = " << pitch << "\n";
		std::cout <<"yaw = " << yaw << "\n\n";
		*/

        std::vector<std::vector<double>> eulerAngles_
        {
            {roll},
            {pitch},
            {yaw}
        };		

		return eulerAngles_;
	}

	void LQServoController::lqi()
	{	
		/*
		// uav1_pose
		eulerAngles = quatToEuler(plant_output.pose.orientation.x, plant_output.pose.orientation.y, plant_output.pose.orientation.z, plant_output.pose.orientation.w);

		Y.at(0).at(0) = plant_output.pose.position.x;
		Y.at(1).at(0) = plant_output.pose.position.y;
		Y.at(2).at(0) = plant_output.pose.position.z;
		Y.at(3).at(0) = eulerAngles.at(0).at(0);
		Y.at(4).at(0) = eulerAngles.at(1).at(0);
		Y.at(5).at(0) = eulerAngles.at(2).at(0);
		*/


		
		// kalman_filter
		Y.at(0).at(0) = estimated_state.x;
		Y.at(1).at(0) = estimated_state.y;
		Y.at(2).at(0) = estimated_state.z;
		Y.at(3).at(0) = estimated_state.phi;
		Y.at(4).at(0) = estimated_state.theta;
		Y.at(5).at(0) = estimated_state.psi;
		
		


		// Estimated State 'X'
        X.at(0).at(0) = Y.at(0).at(0); // x
		X.at(1).at(0) = Y.at(1).at(0); // y
		X.at(2).at(0) = Y.at(2).at(0); // z
		X.at(3).at(0) = Y.at(3).at(0); // roll
		X.at(4).at(0) = Y.at(4).at(0); // pitch
		X.at(5).at(0) = Y.at(5).at(0); // yaw
		X.at(6).at(0) = estimated_state.u_x;
		X.at(7).at(0) = estimated_state.u_y;
		X.at(8).at(0) = estimated_state.u_z;
		X.at(9).at(0) = estimated_state.w_phi;
		X.at(10).at(0) = estimated_state.w_theta;
		X.at(11).at(0) = estimated_state.w_psi;


		/*
		U = -L*X_aug
		X_aug = X + Xi
		Xi = integral(r-E*Y)
		*/

		//printf("\n--------------------------------------------------------------------------------------------------\n");		
        matrice_mul(E, Y, E_times_Y, row_E, column_E, row_Y, column_Y); // E_times_Y = E*Y
		//std::cout << "\n";
		//std::cout << "E = \n";
		//print_mat(E, row_E, column_E);
		//std::cout << "\nY = \n";		
		//print_mat(Y, row_Y, column_Y);
		//std::cout << "\nE_times_Y = \n";
		//print_mat(E_times_Y, row_E, column_Y);
		//std::cout << "\nr = \n";
		//print_mat(r, row_r, column_r);


		// Integrator/Accumulator
		int i = 0;
		for(; i<row_r-1; i++)
		{
			X.at(i).at(0) =  -r.at(i).at(0) + E_times_Y.at(i).at(0); // This loop is only for the xyz
		}		
		X.at(5).at(0) =  -r.at(3).at(0) + E_times_Y.at(3).at(0); // This is for the yaw

		Xi = Xi + r.at(0).at(0) - E_times_Y.at(0).at(0);	// Z-axis
		Yi = Yi + r.at(1).at(0) - E_times_Y.at(1).at(0);	// Z-axis
		Zi = Zi + r.at(2).at(0) - E_times_Y.at(2).at(0);	// Z-axis
		yawi = yawi + r.at(3).at(0) - E_times_Y.at(3).at(0);	// yaw

		
		// Concatenating X and Xi to form X_aug
		i = 0;
		for(;i<row_X; i++)
		{
			X_aug.at(i).at(0) = X.at(i).at(0);
		}
		
		X_aug.at(12).at(0) = Xi;
		X_aug.at(13).at(0) = Yi;
		X_aug.at(14).at(0) = Zi;
		X_aug.at(15).at(0) = yawi;

		//std::cout << "\nX = \n";
		//print_mat(X, row_X, column_X);
		//std::cout << "\nXi = \n";		
		//print_mat(Xi, row_r, column_r);		

		// Calculating the control vector 'U'
		matrice_mul(L, X_aug, U, row_L, column_L, row_X_aug, column_X_aug); // U = L*X_aug
		// U=-L*X_aug
		U.at(0).at(0) = -U.at(0).at(0);
		U.at(1).at(0) = -U.at(1).at(0);
		U.at(2).at(0) = -U.at(2).at(0);
		U.at(3).at(0) = -U.at(3).at(0);

		//std::cout << "\n";
		//std::cout << "\nL = \n";		
		//print_mat(L, row_L, column_L);
		//std::cout << "\nX_aug = \n";		
		//print_mat(X_aug, row_X_aug, column_X_aug);

		//printf("\nactual x = %f\n", plant_output.pose.position.x);
		//printf("actual y = %f\n", plant_output.pose.position.y);
		//printf("actual z = %f\n\n", plant_output.pose.position.z);
		//std::cout << "\nU = \n";		
		//print_mat(U, row_U, column_U);

		control_vector.f_t =  U.at(0).at(0);
		control_vector.tau_x = U.at(1).at(0);
		control_vector.tau_y = U.at(2).at(0);
		control_vector.tau_z = U.at(3).at(0);

		command_pub.publish(control_vector);
		motorSpeeds(U);	
		motor_speeds_pub.publish(motor_speeds);
	}

	void LQServoController::matrice_mul(std::vector<std::vector<double>> &A, std::vector<std::vector<double>> &B, std::vector<std::vector<double>> &result, uint8_t row_a, uint8_t column_a, uint8_t row_b, uint8_t column_b)
	{
		// Check the dimensions of the matrices
		if(column_a != row_b)
		{
			std::cout << "The dimensions of the two matrices do not match: \n";
			std::cout << "Number of columns of the first matrice = %d, Number of rows of the second matrice = %d", column_a, row_b;
		}
		else
		{
			int result_row = row_a;
			int result_column = column_b;
			int i = 0;
			int j = 0;
			int k = 0;
			
			for(; i<result_row; i++)
			{
				for(; j<column_b; j++)
				{
					result[i][j] = 0.0;
					for(; k<row_b; k++)
					{
						result[i][j] = result[i][j] + A[i][k] * B[k][j];
						//ROS_INFO("After result[%d][%d] = %f", i, j, result[i][j]);
					}
					k = 0;
				}
				j = 0;
			}			
		}
	}

	void LQServoController::motorSpeeds(std::vector<std::vector<double>> &control_vector_)
	{
		double m0, m1, m2, m3;
		double l = 0.17;
		double b = 1.858582798e-5; // lift factor
		double d = 7.905545266e-7; // drag factor

		m0 = sqrt(control_vector_.at(0).at(0)/(4*b) - s*control_vector_.at(1).at(0)/(4*b*l) - s*control_vector_.at(2).at(0)/(4*b*l) - control_vector_.at(3).at(0)/(4*d));
		m1 = sqrt(control_vector_.at(0).at(0)/(4*b) + s*control_vector_.at(1).at(0)/(4*b*l) + s*control_vector_.at(2).at(0)/(4*b*l) - control_vector_.at(3).at(0)/(4*d));
		m2 = sqrt(control_vector_.at(0).at(0)/(4*b) + s*control_vector_.at(1).at(0)/(4*b*l) - s*control_vector_.at(2).at(0)/(4*b*l) + control_vector_.at(3).at(0)/(4*d));
		m3 = sqrt(control_vector_.at(0).at(0)/(4*b) - s*control_vector_.at(1).at(0)/(4*b*l) + s*control_vector_.at(2).at(0)/(4*b*l) + control_vector_.at(3).at(0)/(4*d));


		if(isnan(m0))
		{
			m0 = 0.0;
		}
		if(isnan(m1))
		{
			m1 = 0.0;
		}
		if(isnan(m2))
		{
			m2 = 0.0;
		}
		if(isnan(m3))
		{
			m3 = 0.0;
		}

		motor_speeds.angular_velocities[0] = m0;
		motor_speeds.angular_velocities[1] = m1;
		motor_speeds.angular_velocities[2] = m2;
		motor_speeds.angular_velocities[3] = m3;

		//std::cout << "m0 = " << motor_speeds.angular_velocities[0] << "\n";
		//std::cout << "m1 = " << motor_speeds.angular_velocities[1] << "\n";
		//std::cout << "m2 = " << motor_speeds.angular_velocities[2] << "\n";
		//std::cout << "m3 = " << motor_speeds.angular_velocities[3] << "\n";		

	}
	
	void LQServoController::print_mat(std::vector<std::vector<double>> &mat, uint8_t rows, uint8_t columns)
	{
		int i = 0;
		int j = 0;

		for(;i<rows;i++)
		{
			for(;j<columns;j++)
			{
				printf("%.6f, ", mat.at(i).at(j));
			}
			std::cout << "\n";
			j = 0;
		}
	}
}


