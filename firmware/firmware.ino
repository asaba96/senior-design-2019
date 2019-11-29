#include <Wire.h>
#include <ros.h>
#include <std_msgs/Int8.h>
#include <mbz2020_common/MultiRange.h>
#include <VL53L1X.h>

#define PWM_OUTA1 3
#define PWM_OUTA2 13
#define PWM_OUTB1 2
#define PWM_OUTB2 12

#define dir1 8
#define dir2 9

//Current Sensors
#define analogCurrA1 A1
#define analogCurrA2 A3
#define analogCurrB1 A0
#define analogCurrB2 A2

#define MOTOR_ON 1000
#define MOTOR_OFF 0

#define TOF1 6
#define TOF2 11

VL53L1X sensor;
VL53L1X sensor2;

ros::NodeHandle nh;

// whether or not to kill motors
int m_state = 0; 

void set_motors(const int state) {
  analogWrite(PWM_OUTA1, state);
  analogWrite(PWM_OUTA2, state);
  analogWrite(PWM_OUTB1, state);
  analogWrite(PWM_OUTB2, state);
}

//change the motor state
void motors( const std_msgs::Int8& motor_msg) {
  m_state = 0; 
  if (motor_msg.data == 0) {
    //MOTORS OFF
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, HIGH);
    analogWrite(PWM_OUTA1, MOTOR_OFF);
    analogWrite(PWM_OUTA2, MOTOR_OFF);
    analogWrite(PWM_OUTB1, MOTOR_OFF);
    analogWrite(PWM_OUTB2, MOTOR_OFF);
  }
  else if (motor_msg.data == 1) {
    //MOTORS ON - PICKUP MODE
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, HIGH);
    analogWrite(PWM_OUTA1, MOTOR_ON);
    analogWrite(PWM_OUTA2, MOTOR_ON);
    analogWrite(PWM_OUTB1, MOTOR_ON);
    analogWrite(PWM_OUTB2, MOTOR_ON);  
  }
  else {
    //MOTORS ON - DROP MODE
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, LOW);
    analogWrite(PWM_OUTA1, MOTOR_ON);
    analogWrite(PWM_OUTA2, MOTOR_ON);
    analogWrite(PWM_OUTB1, MOTOR_ON);
    analogWrite(PWM_OUTB2, MOTOR_ON);
  }
}

mbz2020_common::MultiRange range_msg;

ros::Subscriber<std_msgs::Int8> sub("motor_cmd", &motors);
ros::Publisher range_pub("multi_range", &range_msg);

void setup() {
  //current sensors
  pinMode(analogCurrA1, INPUT);
  pinMode(analogCurrA2, INPUT);
  pinMode(analogCurrB1, INPUT);
  pinMode(analogCurrB2, INPUT);

  // we have to set PWM pin as output
  pinMode(PWM_OUTA1, OUTPUT);
  pinMode(PWM_OUTA2, OUTPUT);
  pinMode(PWM_OUTB1, OUTPUT);
  pinMode(PWM_OUTB2, OUTPUT);

  // direction pins are also set as output
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);


  //Ros
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(range_pub);

  //Distance Sensors
  pinMode(TOF1, OUTPUT);
  pinMode(TOF2, OUTPUT);

  digitalWrite(TOF1, LOW);
  digitalWrite(TOF2, LOW);

  // Init I2C
  delay(500);
  Wire.begin();
  Wire.beginTransmission(0x29);

  digitalWrite(TOF1, HIGH);
  delay(150);
  sensor.init();
  delay(100);
  sensor.setAddress(0x33);

  digitalWrite(TOF2, HIGH);
  delay(150);
  sensor2.init();
  delay(100);
  sensor2.setAddress(0x35);
  delay(100);

  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(30000);
  sensor.startContinuous(50);
  sensor.setTimeout(100);

  sensor2.setDistanceMode(VL53L1X::Long);
  sensor2.setMeasurementTimingBudget(30000);
  sensor2.startContinuous(50);
  sensor2.setTimeout(100);

  byte count = 0;

  for (byte i = 1; i < 120; i++) {
    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0) {
      count++;
      delay (1);
    } 
  }
}
void loop() {
  // Distance SensorsSensors
  range_msg.range1 = sensor.read();
  range_msg.range_status1 = 1 ? sensor.timeoutOccurred() : 0;

  range_msg.range2 = sensor2.read();
  range_msg.range_status2 = 1 ? sensor2.timeoutOccurred() : 0;

  range_msg.msg_received = nh.now();

  //if current is above or below a certain level, then turn off the motor
  if (analogRead(analogCurrA1) > 550 || analogRead(analogCurrA1) < 460)
  {
    analogWrite(PWM_OUTA1, MOTOR_OFF);
    m_state++;
  }
  if (analogRead(analogCurrA2) > 550 || analogRead(analogCurrA2) < 460)
  {
    analogWrite(PWM_OUTA2, MOTOR_OFF);
    m_state++;
  }
  if (analogRead(analogCurrB1) > 550 || analogRead(analogCurrB1) < 460)
  {
    analogWrite(PWM_OUTB1, MOTOR_OFF);
    m_state++;
  }
  if (analogRead(analogCurrB2) > 550 || analogRead(analogCurrB2) < 460)
  {
    analogWrite(PWM_OUTB2, MOTOR_OFF);
    m_state++;
  }

  range_msg.motor_status = 1 ? m_state > 1 : 0;
  range_pub.publish(&range_msg);

  nh.spinOnce();
}
