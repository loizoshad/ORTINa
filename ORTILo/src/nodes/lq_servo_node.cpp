#include <ORTILo/lq_servo.h>

using namespace lq_servo;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lq_servo_controller");
  LQServoController lq_servo_controller;
  printf("\n-----------------------------------------------------------\nlqi_servo_controller node has been successfully initialized\n-----------------------------------------------------------\n");
  ros::Rate rate(30);

  while (ros::ok())
  { 
    lq_servo_controller.lqi();  
    ros::spinOnce();
    rate.sleep();
  }

  return 0;  
  
}