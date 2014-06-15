#include <ArduinoHardware.h>
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <uarm/Joints.h>

#include <Servo.h>
#include <EEPROM.h>
#include <UF_uArm.h>


UF_uArm uarm_robot;

void jointCallback(const uarm::Joints &command) {
  uarm_robot.setPosition(command.stretch, command.height,
                         command.arm_rot, command.hand_rot);
}

void gripperCallback(const std_msgs::Bool &command) {
  if (command.data) {
    uarm_robot.gripperCatch();
  } else {
    uarm_robot.gripperRelease();
  }
}


ros::NodeHandle nh;
float positions[5] = {0.0};
//char *names[5] = {"ROT", "L", "R", "HAND_ROT", "HAND"};

ros::Subscriber<std_msgs::Bool> gripper_sub("/uarm/gripper", gripperCallback);
ros::Subscriber<uarm::Joints> joint_sub("/uarm/joint_commands", jointCallback);
                                                   
sensor_msgs::JointState joint_state_msg;
ros::Publisher joint_state_pub("/uarm/joint_states", &joint_state_msg);

std_msgs::Bool button_d4_msg;
ros::Publisher button_d4_pub("/uarm/button/d4", &button_d4_msg);

std_msgs::Bool button_d7_msg;
ros::Publisher button_d7_pub("/uarm/button/d7", &button_d7_msg);

void setup() 
{
  uarm_robot.init();

  joint_state_msg.position_length = 5;
  joint_state_msg.position = positions;
  //  joint_state_msg.name_length = 5;
  //  joint_state_msg.name = names; // these causes freeze

  nh.initNode();
  nh.advertise(joint_state_pub);
  nh.advertise(button_d4_pub);
  nh.advertise(button_d7_pub);
  nh.subscribe(gripper_sub);
  nh.subscribe(joint_sub);
  delay(500);
}

void loop()
{
  joint_state_msg.position[0] = (float)uarm_robot.readAngle(SERVO_ROT);
  joint_state_msg.position[1] = (float)uarm_robot.readAngle(SERVO_L);
  joint_state_msg.position[2] = (float)uarm_robot.readAngle(SERVO_R);
  joint_state_msg.position[3] = (float)uarm_robot.readAngle(SERVO_HAND_ROT);
  joint_state_msg.position[4] = (float)uarm_robot.readAngle(SERVO_HAND);
  button_d4_msg.data = (digitalRead(BTN_D4) == 0);
  button_d7_msg.data = (digitalRead(BTN_D7) == 0);
  button_d4_pub.publish(&button_d4_msg);
  button_d7_pub.publish(&button_d7_msg);
  joint_state_pub.publish(&joint_state_msg);
  nh.spinOnce();
  delay(50);
  uarm_robot.gripperDetach();
}
