#include "EncoderMotor.hpp"
#include "tuner/pidTuner.hpp"
#include "INA219.hpp"
#include "ENC28J60.hpp"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <micro_ros_arduino.h>
#include <cstdint>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <micro_ros_utilities/string_utilities.h>
#include <micro_ros_utilities/type_utilities.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <esp32-hal-log.h>
#include <memory>
#include <Wire.h>
#include <EthernetESP32.h>
#include <ArduinoEigenDense.h>


enum priorityLevels {
  PRIORITY_LOWEST = 0,
  PRIORITY_LOW = 1,
  PRIORITY_NORMAL = 2,
  PRIORITY_HIGH = 3,
  PRIORITY_HIGHEST = 4
};


// Pinout
const uint8_t kPinEnc1A = 16;
const uint8_t kPinEnc1B = 17;
const uint8_t kPinMot1Fwd = 18;
const uint8_t kPinMot1Bck = 19;
const uint8_t kPinMot1EnFwd = 2;
const uint8_t kPinMot1EnBck = 4;
const uint8_t kPinEnc2A = 35;
const uint8_t kPinEnc2B = 34;
const uint8_t kPinMot2Fwd = 32;
const uint8_t kPinMot2Bck = 33;
const uint8_t kPinMot2EnFwd = 13;
const uint8_t kPinMot2EnBck = 12;
const uint8_t kPinCSEth = 0;
const uint8_t kPinMOSIEth = 0;
const uint8_t kPinMISOEth = 0;
const uint8_t kPinSCKEth = 0;
const uint8_t kPinIntEth = 0;
const uint8_t kPinResetEth = 0;

// Hardware
const float kPulsePerRev = 11 * 18.8f;
const float kWheelDiameter = 64.0f;   // [mm]
const float kWheelBase = 100.0f;      // [mm]
const float kVelocityRotationalMax = 27.75f;  // [rad/s] Motor max speed (ASSUMED)
const uint8_t kAddressWattmeterLeft = 0x44;
const uint8_t kAddressWattmeterRight = 0x45;
const float kMotorKt = 1.08155f; // [Nm/A]  Motor torque constant (Averaged from stall and rated torque and currents)
const float kMotorJ = 0.6165f;  // [kg*m^2] Motor rotor inertia (ASSUMED)
const IPAddress kIpDevice = IPAddress(192, 168, 1, 101);
const IPAddress kIpGateway = IPAddress(192, 168, 1, 100);
const IPAddress kSubnet = IPAddress(255, 255, 255, 0);
const IPAddress kIpDns = IPAddress(8, 8, 8, 8);
const byte kMacAddress[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
const IPAddress kIpAgent = IPAddress(192, 168, 1, 100);
const uint16_t kPort = 8888;
const uint8_t kFreuencySPIEth = 3;  // [MHz]

// Settings
const float kGainP = 0.44f;
const float kGainI = 2.933f;
const float kGainD = 0.0440f;
const float kAlpha = 0.01f;
const float kBeta = 2.0f;  // Dont change
const float kKappa = 0.0f;
const Eigen::VectorXf kSensorNoise = Eigen::VectorXf::Ones(1) * 0.01f;
const Eigen::MatrixXf kProcessNoise = Eigen::MatrixXf::Identity(2, 2) * 0.1f;
const float kPIDCoeffLowPass = 0.8f;
const uint8_t kPwmResolution = 12;
const uint32_t kPwmFreq = 1E3;
const uint8_t kLoopFreq = 5;


// Globals
std::unique_ptr<EncoderMotor> motorLeft_;
std::unique_ptr<EncoderMotor> motorRight_;
std::shared_ptr<ENC28J60> eth_;
std::shared_ptr<rcl_publisher_t> publisher_;
std::shared_ptr<rcl_subscription_t> subscriber_;
std::shared_ptr<rclc_executor_t> executor_;
std::shared_ptr<rclc_support_t> support_;
std::shared_ptr<rcl_allocator_t> allocator_;
std::shared_ptr<rcl_node_t> node_;
static micro_ros_utilities_memory_conf_t conf_;
std::shared_ptr<SPIClass> spiEth_ = std::make_shared<SPIClass>(HSPI);
const uint16_t kLoopDelayMotor = static_cast<uint16_t>(round(1000.0f / static_cast<float>(kLoopFreq)));
TestOutput results_;
Eigen::Vector3f position_;
Eigen::Quaternionf orientation_;
TaskHandle_t taskMotorControl_;
TaskHandle_t rosSpinTask_;
TaskHandle_t rosTimeSyncTask_;


// Functions
/**
 * @brief Converts wheel velocities to odometry data.
 * 
 * @param angularVelLeft [rad/s] Angular velocity of left wheel
 * @param angularVelRight [rad/s] Angular velocity of right wheel
 * @return [x delta, theta delta] Distance and angle moved.
 */
Eigen::Vector2f calculateOdometry(const float angularVelLeft, 
                                  const float angularVelRight);


/**
 * @brief Set the velocity of both motors based of desired velocity and rotation.
 * 
 * @param velocity [m/s] Linear velocity.
 * @param rotation [rad/s] Angular velocity.
 * @return False if max speed is exceeded.
 */
bool setVelocity(const float velocity,
                 const float rotation);


/**
 * @brief Motor control and odometry update loop.
 * 
 * @param pvParameters 
 */
void taskMotorControl(void *pvParameters);


/**
 * @brief Calls ros spin function.
 * 
 * @param pvParameters Unused.
 */
static void rosSpin(void *pvParameters);


/**
 * @brief Syncs time with ROS host.
 * 
 * @param pvParameters Unused.
 */
static void rosTimeSync(void *pvParameters);


/**
 * @brief Callback for twist messages. Enforces max speed and sets motor setpoints.
 * 
 * @param msgin Incoming ROS message.
 */
void twistCallback(const void *msgin);


// Micro ROS custom interface.
static bool transportOpen(struct uxrCustomTransport * transport) {
  return eth_->udpTransportOpen(transport);
}


static bool transportClose(struct uxrCustomTransport * transport) {
  return eth_->udpTransportClose(transport);
}


static size_t transportWrite(struct uxrCustomTransport * transport, const uint8_t * buf, size_t len, uint8_t * err) {
  return eth_->udpTransportWrite(transport, buf, len, err);
}


static size_t transportRead(struct uxrCustomTransport * transport, uint8_t * buf, size_t len, int timeout, uint8_t * err) {
  return eth_->udpTransportRead(transport, buf, len, timeout, err);
}


void IRAM_ATTR onClockLeft() {
  motorLeft_->onClock();
}

void IRAM_ATTR onClockRight() {
  motorRight_->onClock();
}


void setup() {
  Serial.begin(115200);
  delay(3000);  // Delay for 3 seconds to allow for reprogramming.
  
  while (!Serial) {
    delay(100);
  }
  
  log_d("Serial Started");
  log_d("Starting Wire...");
  Wire.begin();
  log_d("Wire Started");
  log_d("Starting UDP...");
  spiEth_->begin(kPinSCKEth, kPinMISOEth, kPinMOSIEth, kPinCSEth);
  eth_ = std::make_shared<ENC28J60>(kPinCSEth, kPinIntEth, kPinResetEth, spiEth_, kFreuencySPIEth);
  byte mac[6];

  for (size_t i = 0; i < 6; i++) {
    mac[i] = kMacAddress[i];
  }

  eth_->begin(kIpDevice, kIpGateway, kSubnet, kIpDns, mac, kIpAgent, kPort);
  log_d("UDP started");
  log_d("Starting motors...");

  motorLeft_ = std::make_unique<EncoderMotor>(kPinMot1Fwd, 
                                              kPinMot1Bck, 
                                              kPinMot1EnFwd, 
                                              kPinMot1EnBck, 
                                              kPinEnc1A, 
                                              kPinEnc1B);

  motorLeft_->initialiseMotor(kPwmResolution, kPwmFreq);
  motorLeft_->initialiseUKF(kMotorKt, kMotorJ, kAlpha, kBeta, kKappa, kSensorNoise, kProcessNoise);
  motorLeft_->setPulsePerRev(kPulsePerRev);
  motorLeft_->pidSetGains(kGainP, kGainI, kGainD);
  motorLeft_->initialiseWattmeter(kAddressWattmeterLeft, 0.1, 0.1);
  motorLeft_->pidSetFilterCoeff(kPIDCoeffLowPass);

  motorRight_ = std::make_unique<EncoderMotor>(kPinMot2Fwd, 
                                               kPinMot2Bck, 
                                               kPinMot2EnFwd,
                                               kPinMot2EnBck, 
                                               kPinEnc2A, 
                                               kPinEnc2B);

  motorRight_->initialiseMotor(kPwmResolution, kPwmFreq);
  motorRight_->initialiseUKF(kMotorKt, kMotorJ, kAlpha, kBeta, kKappa, kSensorNoise, kProcessNoise);
  motorRight_->setPulsePerRev(kPulsePerRev);
  motorRight_->pidSetGains(kGainP, kGainI, kGainD);
  motorRight_->initialiseWattmeter(kAddressWattmeterRight, 0.1, 0.1);
  motorRight_->pidSetFilterCoeff(kPIDCoeffLowPass);
  
  attachInterrupt(digitalPinToInterrupt(kPinEnc1A), onClockLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kPinEnc2A), onClockRight, CHANGE);
  motorRight_->setSpeed(0.0);
  motorLeft_->setSpeed(0.0);
  motorRight_->disable();
  motorLeft_->disable();
  log_d("Motors started");
  position_ = Eigen::Vector3f::Zero(3);
  log_d("Starting motor control task...");

  BaseType_t success = xTaskCreate(taskMotorControl,
                                  "motorControl",
                                  4096,
                                  nullptr,
                                  priorityLevels::PRIORITY_HIGH,
                                  &taskMotorControl_);

  if (!success) {
    log_e("Main: Failed to create motor control task");

    while(1) {
      delay(10000);
    }
  }


  log_d("Starting ROS");

  auto passGateRos = [] (const rcl_ret_t result, const std::string string) {
    if (result != RCL_RET_OK) {
      log_e("%s", string.c_str());

      while (1) {
        delay(10000);
      }
    }
  };

  rmw_uros_set_custom_transport(false, 
                                NULL, 
                                transportOpen, 
                                transportClose, 
                                transportWrite, 
                                transportRead);

  set_microros_transports();
  allocator_ = std::make_shared<rcl_allocator_t>();
  support_ = std::make_shared<rclc_support_t>();
  node_ = std::make_shared<rcl_node_t>();
  executor_ = std::make_shared<rclc_executor_t>();
  publisher_ = std::make_shared<rcl_publisher_t>();
  *allocator_ = rcl_get_default_allocator();

  rcl_ret_t error;
  error = rclc_support_init(support_.get(), 
                            0, 
                            NULL, 
                            allocator_.get());

  passGateRos(error, "ROS support init failed");
  log_d("ROS support init success");

  error = rclc_node_init_default(node_.get(), 
                                "micro_ros_arduino_node", 
                                "", 
                                support_.get());

  passGateRos(error, "ROS node init failed");
  log_d("ROS node init success");

  error = rclc_publisher_init_default(publisher_.get(),
                                      node_.get(),
                                      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),  // message type
                                      "micro_ros_arduino_node_publisher");

  passGateRos(error, "ROS publisher init failed");
  log_d("ROS publisher init success");

  error = rclc_executor_init(executor_.get(), 
                            &support_->context, 
                            1, 
                            allocator_.get());

  passGateRos(error, "ROS executor init failed");
  log_d("ROS executor init success");

  success = xTaskCreate(rosSpin,
                        "rosSpin",
                        4096,
                        nullptr,
                        priorityLevels::PRIORITY_HIGHEST,
                        &rosSpinTask_);
                      
  passGateRos(error, "Main: Failed to create dataProcessing task");
  log_d("ROS spin task started");

  success = xTaskCreate(rosTimeSync,
                        "rosTimeSync",
                        4096,
                        nullptr,
                        priorityLevels::PRIORITY_LOW,
                        &rosTimeSyncTask_);

  passGateRos(error, "Main: Failed to create dataProcessing task");
  log_d("ROS spin task started");
}


void loop() {
  vTaskDelete(NULL); // Delete main loop task
}


Eigen::Vector2f calculateOdometry(const float angularVelLeft, 
                                  const float angularVelRight) 
{
  float velocityLeft = angularVelLeft * kWheelDiameter / 2000.0f;
  float velocityRight = angularVelRight * kWheelDiameter / 2000.0f;

  Eigen::Vector2f output;
  output(0) = (kWheelDiameter / 2000.0f) * (velocityLeft + velocityRight);
  output(2) = (kWheelDiameter / kWheelBase) * (angularVelRight - angularVelLeft);

  return output;
}


bool setVelocity(const float velocity,
                 const float rotation) 
{
  // Calculate wheel velocities.
  float velocityLeft = (velocity - (kWheelBase / 2000.0f) * rotation) / (kWheelDiameter / 2000.0f);
  float angularLeft = 2000.0f * velocityLeft / kWheelDiameter;
  float velocityRight = (velocity + (kWheelBase / 2.0f) * rotation) / (kWheelDiameter / 2.0f);
  float angularRight = 2000.0f * velocityRight / kWheelDiameter;
  
  // Enforce max speed.
  if (std::abs(angularLeft) > kVelocityRotationalMax || std::abs(angularRight) > kVelocityRotationalMax) {
    return false;
  }

  motorLeft_->setSpeed(angularLeft);
  motorRight_->setSpeed(angularRight);
  return true;
}


void taskMotorControl(void *pvParameters) {
  motorLeft_->enable();
  motorRight_->enable();
  nav_msgs__msg__Odometry odometryMsg;
  odometryMsg.header.frame_id = micro_ros_string_utilities_init("base_link");
  TickType_t lastWakeTime = xTaskGetTickCount();

  auto rot2Quat = [] (const float theta) {
    Eigen::Quaternionf quat;
    quat.w() = cos(theta / 2);
    quat.x() = 0;
    quat.y() = 0;
    quat.z() = sin(theta / 2);
    return quat;
  };

  for (;;) {
    TickType_t currentTick = xTaskGetTickCount();
    float deltaTime = portTICK_PERIOD_MS * static_cast<float>(currentTick - lastWakeTime) / 1000.0f;
    Eigen::Vector2f stepsLeft = motorLeft_->update(deltaTime);
    Eigen::Vector2f stepsRight = motorRight_->update(deltaTime);
    float velocityLeft = stepsLeft(0) / deltaTime;
    float velocityRight = stepsRight(0) / deltaTime;

    // Update odometry
    Eigen::Vector2f odoChange = calculateOdometry(velocityLeft, velocityRight);
    Eigen::Quaternionf rotationQuat = rot2Quat(odoChange(1));
    orientation_ = orientation_ * rotationQuat;
    Eigen::Vector3f positionVector;
    positionVector(0) = odoChange(0);
    positionVector = orientation_ * positionVector;
    position_ += positionVector;

    // publish odometry
    odometryMsg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
    odometryMsg.header.stamp.nanosec = rmw_uros_epoch_nanos();
    odometryMsg.pose.pose.position.x = position_(0);
    odometryMsg.pose.pose.position.y = position_(1);
    odometryMsg.pose.pose.position.z = position_(2);
    odometryMsg.pose.pose.orientation.w = orientation_.w();
    odometryMsg.pose.pose.orientation.x = orientation_.x();
    odometryMsg.pose.pose.orientation.y = orientation_.y();
    odometryMsg.pose.pose.orientation.z = orientation_.z();
    odometryMsg.twist.twist.linear.x = odoChange(0);
    odometryMsg.twist.twist.angular.z = odoChange(1);

    if (rcl_publish(publisher_.get(), &odometryMsg, NULL) != RCL_RET_OK) {
      log_e("Failed to publish odometry data");
    }
    
    xTaskDelayUntil(&lastWakeTime, kLoopDelayMotor);
  }
}


static void rosSpin(void *pvParameters) {
  log_d("main: ROS spin task started");
  const TickType_t taskDelayPeriod = 100; // [ms].
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&lastWakeTime, taskDelayPeriod);
    rclc_executor_spin_some(executor_.get(), RCL_MS_TO_NS(100));
  }
}


static void rosTimeSync(void *pvParameters) {
  log_d("main: ROS time sync task started");
  const TickType_t taskDelayPeriod = 60000; // [ms].
  const int16_t timeout = 1000; // [ms]. 
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    if (rmw_uros_sync_session(timeout) != RMW_RET_OK) {
      log_e("ROS time sync failed");
    }

    vTaskDelayUntil(&lastWakeTime, taskDelayPeriod);
  }
}


void twistCallback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  // Calculate wheel velocities.
  float velocityLeft = (msg->linear.x - (kWheelBase / 2000.0f) * msg->angular.z) / (kWheelDiameter / 2000.0f);
  float angularLeft = 2000.0f * velocityLeft / kWheelDiameter;
  float velocityRight = (msg->linear.x + (kWheelBase / 2.0f) * msg->angular.z) / (kWheelDiameter / 2.0f);
  float angularRight = 2000.0f * velocityRight / kWheelDiameter;
  
  // Enforce max speed.
  if (std::abs(angularLeft) > kVelocityRotationalMax || std::abs(angularRight) > kVelocityRotationalMax) {
    if (angularLeft - angularRight <= 0.01) {
      angularLeft = std::copysign(kVelocityRotationalMax, angularLeft);
      angularRight = std::copysign(kVelocityRotationalMax, angularRight);

    } else {
      float max = std::max(angularLeft, angularRight);
      float ratio = kVelocityRotationalMax / max;
      angularLeft *= ratio;
      angularRight *= ratio;
    }
  }

  motorLeft_->setSpeed(angularLeft);
  motorRight_->setSpeed(angularRight);
}