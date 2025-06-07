#include <Servo.h>
#include <ros.h>
#include <std_msgs/String.h>

#define UPPER_BRUSHLESS_PIN 5
#define LOWERL_BRUSHLESS_PIN 6
#define LOWERR_BRUSHLESS_PIN 7
#define DC_MOTOR_DIR_PIN 8
#define DC_MOTOR_ENABLE_PIN 9
#define SERVO_PIN 3
#define ROTATION_DELAY 1000
#define SERVO_INIT_ANGLE 90

Servo UpperESC;
Servo LowerLESC;
Servo LowerRESC;
Servo myservo;

ros::NodeHandle nh;

void motorSpeedsCallback(const std_msgs::String& msg) {
  String command = msg.data;
  int delimiterIndex1 = command.indexOf(',');
  int delimiterIndex2 = command.lastIndexOf(',');

  if (delimiterIndex1 == -1 || delimiterIndex2 == -1 || delimiterIndex1 == delimiterIndex2) {
    nh.logwarn("Received invalid speed format");
    return;
  }

  int upperVal = command.substring(0, delimiterIndex1).toInt();
  int lowerlVal = command.substring(delimiterIndex1 + 1, delimiterIndex2).toInt();
  int lowerrVal = command.substring(delimiterIndex2 + 1).toInt();

  UpperESC.writeMicroseconds(upperVal);
  LowerLESC.writeMicroseconds(lowerlVal);
  LowerRESC.writeMicroseconds(lowerrVal);

  nh.loginfo("Updated motor speeds");
}

void dcMotorContinuousControlCallback(const std_msgs::String& msg) {
  String input = String(msg.data);

  if (input.equals("start")) {
    digitalWrite(DC_MOTOR_DIR_PIN, HIGH); // Set direction to clockwise
    digitalWrite(DC_MOTOR_ENABLE_PIN, HIGH); // Enable motor for continuous rotation
    nh.loginfo("DC Motor started continuous rotation");
    delay(ROTATION_DELAY); // Delay between rotations
  } else if (input.equals("stop")) {
    digitalWrite(DC_MOTOR_ENABLE_PIN, LOW); // Stop motor immediately
    nh.loginfo("DC Motor stopped continuous rotation");
  } else {
    nh.logwarn("Invalid command for DC motor. Use 'start' or 'stop'.");
  }
}

void servoAngleCallback(const std_msgs::String& msg) {
  int angle = atoi(msg.data);
  myservo.write(angle);
  nh.loginfo("Updated servo angle");
}

ros::Subscriber<std_msgs::String> motorSpeedSub("motor_speeds_topic", &motorSpeedsCallback);
ros::Subscriber<std_msgs::String> dcMotorControlSub("dc_motor_control_topic", &dcMotorContinuousControlCallback);
ros::Subscriber<std_msgs::String> servoAngleSub("servo_angle_topic", &servoAngleCallback);

void setup() {
  UpperESC.attach(UPPER_BRUSHLESS_PIN, 1000, 2000);
  LowerLESC.attach(LOWERL_BRUSHLESS_PIN, 1000, 2000);
  LowerRESC.attach(LOWERR_BRUSHLESS_PIN, 1000, 2000);
  myservo.attach(SERVO_PIN);

  pinMode(DC_MOTOR_DIR_PIN, OUTPUT);
  pinMode(DC_MOTOR_ENABLE_PIN, OUTPUT);

  UpperESC.writeMicroseconds(1000);
  LowerLESC.writeMicroseconds(1000);
  LowerRESC.writeMicroseconds(1000);
  myservo.write(SERVO_INIT_ANGLE);

  nh.initNode();
  nh.subscribe(motorSpeedSub);
  nh.subscribe(dcMotorControlSub);
  nh.subscribe(servoAngleSub);
}

void loop() {
  nh.spinOnce();
}
