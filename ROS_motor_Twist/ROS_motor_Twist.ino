#include <ros.h>
#include <Arduino.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>


const int IN1 = 3;
const int IN2 = 4;
const int IN3 = 5;
const int IN4 = 6;
const int ENA = A0;
const int ENB = A1;

const float d = 0.2f; // ボディ中心からタイヤまでの距離
const float max_velo = 1.0f; // ローガーが出せる最大の速度

ros::NodeHandle nh;
void left_motor(int duty_l);
void right_motor(int duty_r);

void vel_cb(const geometry_msgs::Twist& msg) {
  float linearX = msg.linear.x;
  float angularZ = msg.angular.z;

  float leftDuty = linearX - angularZ;
  float rightDuty = linearX + angularZ;

  left_motor(leftDuty * 255);
  right_motor(rightDuty * 255);

}

ros::Subscriber<geometry_msgs::Twist> sub_mt("/cmd_vel", &vel_cb);
void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub_mt);
}

void loop() {
  nh.spinOnce();
  delay(1);
}



void left_motor(int duty_l) {
  int code;
  char buf[100];
  sprintf(buf, "duty: Left %d ", duty_l);
  nh.loginfo(buf);

  if (abs(duty_l) >= 255) {
    if (duty_l < 0)code = -1; else code = 1;
    duty_l = 255 * code;
  }

  if (duty_l > 0) {
    analogWrite(ENB, abs(duty_l));
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (duty_l < 0) {
    analogWrite(ENB, abs(duty_l));
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    analogWrite(ENB, 0);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
}

void right_motor(int duty_r) {
  int code;
  char buf[100];
  sprintf(buf, "duty: Right %d ", duty_r);
  nh.loginfo(buf);

  if (abs(duty_r) >= 255) {
    if (duty_r < 0)code = -1; else code = 1;
    duty_r = 255 * code;
  }

  if (duty_r > 0) {
    analogWrite(ENA, abs(duty_r));
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (duty_r < 0) {
    analogWrite(ENA, abs(duty_r));
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    analogWrite(ENA, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
}
