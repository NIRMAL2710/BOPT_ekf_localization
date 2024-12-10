
///##########  This program is to get the encoder and imu value 
///##########  and publish it in the Ros 
///##########  various topics Sterring angle;Movement Velocity;IMU value 


#include "MPU9250.h"
#include <Wire.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>

// MPU9250 instance
MPU9250 mpu;

// ROS NodeHandle
ros::NodeHandle nh;

// ROS IMU Publisher
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu_data", &imu_msg);

// ROS Publishers for movement and steering
std_msgs::Float32 movementVelocityMsg;
std_msgs::Float32 steeringAngleMsg;
ros::Publisher movementVelocityPub("movement_velocity", &movementVelocityMsg);
ros::Publisher steeringAnglePub("steering_angle", &steeringAngleMsg);

// Encoder pins for Movement Motor (Motor A)
const int encoderPinA1 = 7; // Teensy 3.5 interrupt-capable pins
const int encoderPinB1 = 6; // Teensy 3.5 interrupt-capable pins

// Encoder pins for Steering Motor (Motor B)
const int encoderPinA2 = 5; // Teensy 3.5 interrupt-capable pins
const int encoderPinB2 = 4; // Teensy 3.5 interrupt-capable pins

// Encoder parameters for Motor A (Movement)
const long PPR1 = 4100;  // PPR for Movement Motor (Motor A)
const float wheelRadius = 0.13;  // Wheel radius in meters (26 cm diameter)

// Encoder parameters for Motor B (Steering)
const long PPR2 = 262000;  // PPR for Steering Motor (Motor B)
const float steeringGearRatio = 1.0; // Change if there is a steering gear ratio

// Variables to store encoder data for Motor A (Movement)
volatile long pulseCount1 = 0;
long previousPulseCount1 = 0;
unsigned long previousTime1 = 0;
volatile long lastEncoded1 = 0;

// Variables to store encoder data for Motor B (Steering)
volatile long pulseCount2 = 0;
long previousPulseCount2 = 0;
unsigned long previousTime2 = 0;
volatile long lastEncoded2 = 0;

void setup() {
    // Begin serial communication at 115200 baud rate
    Serial.begin(115200);
    Wire.begin();
    delay(2000);

    // Initialize ROS node and publishers
    nh.initNode();
    nh.advertise(movementVelocityPub);
    nh.advertise(steeringAnglePub);
    nh.advertise(imu_pub);

    // Initialize MPU9250
    if (!mpu.setup(0x68)) {  // Check and change to your device's I2C address if needed
        while (1) {
            Serial.println("MPU connection failed. Please check your wiring.");
            delay(5000);
        }
    } else {
        Serial.println("MPU9250 connected!");
    }

    // Set encoder pins for Motor A (Movement) as input with pull-up resistors
    pinMode(encoderPinA1, INPUT_PULLUP);
    pinMode(encoderPinB1, INPUT_PULLUP);

    // Set encoder pins for Motor B (Steering) as input with pull-up resistors
    pinMode(encoderPinA2, INPUT_PULLUP);
    pinMode(encoderPinB2, INPUT_PULLUP);

    // Attach interrupts to the encoder pins for Motor A (Movement)
    attachInterrupt(digitalPinToInterrupt(encoderPinA1), updateEncoder1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPinB1), updateEncoder1, CHANGE);

    // Attach interrupts to the encoder pins for Motor B (Steering)
    attachInterrupt(digitalPinToInterrupt(encoderPinA2), updateEncoder2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPinB2), updateEncoder2, CHANGE);

    // Initial time for both motors
    previousTime1 = millis();
    previousTime2 = millis();
}

void loop() {
    // Handle Motor A (Movement)
    unsigned long currentTime1 = millis();
    unsigned long timeElapsed1 = currentTime1 - previousTime1; // Time difference in milliseconds

    if (timeElapsed1 >= 500) {  // Calculate speed every 0.5 seconds for Motor A
        long currentPulseCount1 = pulseCount1;
        long pulseDifference1 = currentPulseCount1 - previousPulseCount1;

        // Calculate angular velocity in radians per second for Motor A
        float angularVelocity1 = (2 * PI * pulseDifference1) / (PPR1 * (timeElapsed1 / 1000.0));

        // Calculate linear velocity (m/s) for Motor A
        float linearVelocity = angularVelocity1 * wheelRadius;
        Serial.println(linearVelocity);
        // Publish encoder data for Motor A
        movementVelocityMsg.data = linearVelocity;
        movementVelocityPub.publish(&movementVelocityMsg);

        // Update previous values for Motor A
        previousPulseCount1 = currentPulseCount1;
        previousTime1 = currentTime1;
    }

    // Handle Motor B (Steering)
    long currentPulseCount2 = pulseCount2;

    // Calculate steering angle (in radians) for Motor B
    float steeringAngle = (2.0 * PI * currentPulseCount2) / (PPR2 * steeringGearRatio);

    // Publish steering angle for Motor B
    steeringAngleMsg.data = steeringAngle; // Publish steering angle
    steeringAnglePub.publish(&steeringAngleMsg);
    Serial.println(steeringAngle);

    // Handle MPU9250 IMU Data
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 100) {  // Update data every 100 ms
            publish_imu_data();
            prev_ms = millis();
        }
    }

    // Keep ROS node alive
    nh.spinOnce();
}

void publish_imu_data() {
    // Fill IMU message with accelerometer data
    imu_msg.linear_acceleration.x = mpu.getAccX();
    imu_msg.linear_acceleration.y = mpu.getAccY();
    imu_msg.linear_acceleration.z = mpu.getAccZ();

    // Fill IMU message with gyroscope data
    imu_msg.angular_velocity.x = mpu.getGyroX();
    imu_msg.angular_velocity.y = mpu.getGyroY();
    imu_msg.angular_velocity.z = mpu.getGyroZ();

    // Convert orientation (Yaw, Pitch, Roll) to quaternion
    float qx, qy, qz, qw;
    eulerToQuaternion(mpu.getRoll(), mpu.getPitch(), mpu.getYaw(), qx, qy, qz, qw);
    imu_msg.orientation.x = qx;
    imu_msg.orientation.y = qy;
    imu_msg.orientation.z = qz;
    imu_msg.orientation.w = qw;

    // Publish the IMU data on the "imu_data" topic
    imu_pub.publish(&imu_msg);
}

// Interrupt service routine for Motor A (Movement)
void updateEncoder1() {
    int MSB = digitalRead(encoderPinA1); // MSB = most significant bit
    int LSB = digitalRead(encoderPinB1); // LSB = least significant bit

    int encoded = (MSB << 1) | LSB; // Combine the two bits into a single value
    int sum = (lastEncoded1 << 2) | encoded; // Add it to the previous encoded value

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) pulseCount1++;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) pulseCount1--;

    lastEncoded1 = encoded; // Store the current encoded value for next time
}

// Interrupt service routine for Motor B (Steering)
void updateEncoder2() {
    int MSB = digitalRead(encoderPinA2); // MSB = most significant bit
    int LSB = digitalRead(encoderPinB2); // LSB = least significant bit

    int encoded = (MSB << 1) | LSB; // Combine the two bits into a single value
    int sum = (lastEncoded2 << 2) | encoded; // Add it to the previous encoded value

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) pulseCount2++;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) pulseCount2--;

    lastEncoded2 = encoded; // Store the current encoded value for next time
}

void eulerToQuaternion(float roll, float pitch, float yaw, float &qx, float &qy, float &qz, float &qw) {
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    qw = cr * cp * cy + sr * sp * sy;
    qx = sr * cp * cy - cr * sp * sy;
    qy = cr * sp * cy + sr * cp * sy;
    qz = cr * cp * sy - sr * sp * cy;
}
