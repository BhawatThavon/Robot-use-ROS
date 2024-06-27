#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <Encoder.h>

const int dirPins[] = {4, 6};
const int pwmPins[] = {5, 7};
const int encoderAPins[] = {2, 3};
const int encoderBPins[] = {A0, A1};
const int over = 22;
const int under = 24;
const int error = 26;
int under2;
int over2;
int error2;
int front_left_pwm = 0;
int front_right_pwm = 0;
float front_left_speed = 0.0;
float front_right_speed = 0.0;

ros::NodeHandle nh;

Encoder myEnc[] = {
  Encoder(encoderAPins[0], encoderBPins[0]),
  Encoder(encoderAPins[1], encoderBPins[1]),
};

std_msgs::Int32 front_left_ticks_callback;
std_msgs::Int32 front_right_ticks_callback;

ros::Publisher front_left_encoder_pub("front_left_encoder_ticks", &front_left_ticks_callback);
ros::Publisher front_right_encoder_pub("front_right_encoder_ticks", &front_right_ticks_callback);

const float track_width = 0.48;

void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel) {
  float linear_velocity_x = cmd_vel.linear.x;
  float linear_velocity_y = cmd_vel.linear.y;
  float angular_velocity_z = cmd_vel.angular.z;

  front_left_speed = linear_velocity_x - linear_velocity_y - angular_velocity_z; // * (track_width / 2.0)
  front_right_speed = linear_velocity_x + linear_velocity_y + angular_velocity_z;
//  Serial.print("front left speed =");
//  Serial.println(front_left_speed);
//  Serial.print("front right speed =");
//  Serial.println(front_right_speed);

  front_left_pwm = int(front_left_speed * 255.0 / 0.37);
  front_right_pwm = int(front_right_speed * 255.0 / 0.37);
//  Serial.print("front left pwm =");
//  Serial.println(front_left_pwm);
//  Serial.print("front right pwm =");
//  Serial.println(front_right_pwm);
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel_scaled", &cmd_vel_callback); //cmd_vel cmd_vel_scaled

void setup() {
  for (int i = 0; i < 2; i++) {
    pinMode(dirPins[i], OUTPUT);
    pinMode(pwmPins[i], OUTPUT);
    pinMode(over, INPUT);
    pinMode(under, INPUT);
    pinMode(error, INPUT);
    digitalWrite(under, HIGH);
  }

  nh.initNode();
  nh.advertise(front_left_encoder_pub);
  nh.advertise(front_right_encoder_pub);
  nh.subscribe(cmd_vel_sub);
  Serial.begin(57600);
  nh.getHardware()->setBaud(57600);
}

void loop() {
 
  if (digitalRead(over) == HIGH || digitalRead(under) == HIGH || digitalRead(error) == HIGH) {
    front_left_pwm = 0;
    front_right_pwm = 0;
//    Serial.println("Stop");
  }

//  under2 = digitalRead(under);
//  Serial.print("under: ");
//  Serial.println(under2);
//  Serial.print("over: ");
//  Serial.println(over2);
//  Serial.print("error: ");
//  Serial.println(error2);

  front_left_ticks_callback.data = myEnc[0].read();
  front_right_ticks_callback.data = myEnc[1].read();
//  float back_left = myEnc[1].read();
//  Serial.println(back_left);
  
  front_left_encoder_pub.publish(&front_left_ticks_callback);
  front_right_encoder_pub.publish(&front_right_ticks_callback);

  analogWrite(pwmPins[0], abs(front_left_pwm));
  digitalWrite(dirPins[0], front_left_pwm > 0 ? LOW : HIGH);
  analogWrite(pwmPins[1], abs(front_right_pwm));
  digitalWrite(dirPins[1], front_right_pwm > 0 ? LOW : HIGH);
  nh.spinOnce();
}
