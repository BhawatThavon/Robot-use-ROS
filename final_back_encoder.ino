#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <Encoder.h>

const int dirPins[] = {8, 10};
const int pwmPins[] = {9, 11};
const int encoderAPins[] = {2, 3};
const int encoderBPins[] = {A2, A3};
const int over = 22;
const int under = 24;
const int error = 26;
int under2;
int over2;
int error2;
int back_left_pwm = 0;
int back_right_pwm = 0;
float back_left_speed = 0.0;
float back_right_speed = 0.0;

ros::NodeHandle nh;

Encoder myEnc[] = {
  Encoder(encoderAPins[0], encoderBPins[0]),
  Encoder(encoderAPins[1], encoderBPins[1])
};

std_msgs::Int32 back_left_ticks_callback;
std_msgs::Int32 back_right_ticks_callback;

ros::Publisher back_left_encoder_pub("back_left_encoder_ticks", &back_left_ticks_callback);
ros::Publisher back_right_encoder_pub("back_right_encoder_ticks", &back_right_ticks_callback);

const float track_width = 0.48;

void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel) {
  float linear_velocity_x = cmd_vel.linear.x;
  float linear_velocity_y = cmd_vel.linear.y;
  float angular_velocity_z = cmd_vel.angular.z;

  back_left_speed = linear_velocity_x + linear_velocity_y - angular_velocity_z;
  back_right_speed = linear_velocity_x - linear_velocity_y + angular_velocity_z;
//  Serial.print("back left speed =");
//  Serial.println(back_left_speed);
//  Serial.print("back right speed =");
//  Serial.println(back_right_speed);

  back_left_pwm = int(back_left_speed * 255.0 / 0.37);
  back_right_pwm = int(back_right_speed * 255.0 / 0.37);
//  Serial.print("back left pwm =");
//  Serial.println(back_left_pwm);
//  Serial.print("back right pwm =");
//  Serial.println(back_right_pwm);
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
  nh.advertise(back_left_encoder_pub);
  nh.advertise(back_right_encoder_pub);
  nh.subscribe(cmd_vel_sub);
  Serial.begin(57600);
  nh.getHardware()->setBaud(57600);
}

void loop() {

  if (digitalRead(over) == HIGH || digitalRead(under) == HIGH || digitalRead(error) == HIGH) {
    back_left_pwm = 0;
    back_right_pwm = 0;
//    Serial.println("Stop");
  }

//  under2 = digitalRead(under);
//  Serial.print("under: ");
//  Serial.println(under2);
//  over2 = digitalRead(over);
//  Serial.print("over: ");
//  Serial.println(over2);
//  error2 = digitalRead(error);
//  Serial.print("error: ");
//  Serial.println(error2);

  back_left_ticks_callback.data = myEnc[0].read();
  back_right_ticks_callback.data = myEnc[1].read();
//  float back_left = myEnc[1].read();
//  Serial.println(back_left);

  back_left_encoder_pub.publish(&back_left_ticks_callback);
  back_right_encoder_pub.publish(&back_right_ticks_callback);

  analogWrite(pwmPins[0], abs(back_left_pwm));
  digitalWrite(dirPins[0], back_left_pwm > 0 ? LOW : HIGH);
  analogWrite(pwmPins[1], abs(back_right_pwm));
  digitalWrite(dirPins[1], back_right_pwm > 0 ? LOW : HIGH);
  nh.spinOnce();
}
