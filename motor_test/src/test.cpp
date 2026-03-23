#include "EncoderMotor.hpp"
#include "tuner/pidTuner.hpp"
#include "INA219.hpp"
#include "common/helperFunctions.hpp"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <memory>
#include <deque>
#include <Wire.h>
#include <esp32-hal-log.h>


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

// Hardware
const float kPulsePerRev = 11 * 18.8f;
const float kWheelDiameter = 64.0f;  // [mm]
const float kWheelBase = 100.0f;      // [mm]
const float kVelocityRotationalMax = 27.75;  // [rad/s] Motor max speed (ASSUMED)
const uint8_t kAddressWattmeterLeft = 0x44;
const uint8_t kAddressWattmeterRight = 0x45;
const float kMotorKt = 1.08155f; // [Nm/A]  Motor torque constant (Averaged from stall and rated torque and currents)
const float kMotorJ = 0.6165;  // [kg*m^2] Motor rotor inertia (ASSUMED)

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
const uint8_t kLoopFreq = 10;

std::unique_ptr<IBT2> ibt2_;
std::unique_ptr<IBT2> ibt22_;
std::unique_ptr<Encoder> encoder_;
std::unique_ptr<Encoder> encoder2_;
std::unique_ptr<INA219Local> ina219_;
std::unique_ptr<INA219Local> ina2192_;
std::unique_ptr<UKF> ukf_;
std::unique_ptr<UKF> ukf2_;
std::unique_ptr<PID> pid_;
std::unique_ptr<PID> pid2_;
LowPassExponential lowPass_(0.3f);
LowPassExponential lowPass2_(0.8f);
float stepAngle_;

const size_t kTestDuration = 9; // [s]
uint32_t millisPrevLeft_ = 0;
uint32_t millisPrevRight_ = 0;
uint16_t loopDelayMs_ = static_cast<uint16_t>(round(1000.0 / static_cast<float>(kLoopFreq)));

const uint16_t windowSize_ = (kTestDuration * 1E3) / loopDelayMs_;
TaskHandle_t taskHandle_;
TaskHandle_t endHandle_;

/// Test functions
void task(void *pvParameters);
void taskResults(void *pvParameters);

void IRAM_ATTR onClockLeft() {
  encoder_->onClock();
}

void IRAM_ATTR onClockRight() {
  encoder2_->onClock();
}

void setup() {
  Serial.begin(115200);
  delay(3000);
  Wire.begin();

  encoder_ = std::make_unique<Encoder>(kPinEnc1A, kPinEnc1B);
  encoder2_ = std::make_unique<Encoder>(kPinEnc2A, kPinEnc2B);
  ina219_ = std::make_unique<INA219Local>(kAddressWattmeterLeft);
  ina219_->calibrate(ina226_average_enum::INA226_1024_SAMPLES,
                    ina226_timing_enum::INA226_140_us,
                    ina226_timing_enum::INA226_140_us,
                    0.1f,
                    0.1f);
  ina2192_ = std::make_unique<INA219Local>(kAddressWattmeterRight);
  ina2192_->calibrate(ina226_average_enum::INA226_1024_SAMPLES,
                    ina226_timing_enum::INA226_140_us,
                    ina226_timing_enum::INA226_140_us,
                    0.1f,
                    0.1f);
  ibt2_ = std::make_unique<IBT2>(kPinMot1Fwd, kPinMot1Bck, kPinMot1EnFwd, kPinMot1EnBck);
  ibt22_ = std::make_unique<IBT2>(kPinMot2Fwd, kPinMot2Bck, kPinMot2EnFwd, kPinMot2EnBck);
  UKFModel model(2, 1);
  model.stateTransition = [] (const Eigen::VectorXf& state, const Eigen::VectorXf& control, float deltaSec) {
    float pos = state(0);
    float vel = state(1);
    float current = control(0);
    float accel = (kMotorKt * current) / kMotorJ;

    Eigen::VectorXf nextState(2);
    nextState(0) = pos + (vel * deltaSec) + (0.5 * accel * deltaSec * deltaSec);
    nextState(1) = vel + (accel * deltaSec);
    return nextState;
  };

  model.measurementModel = [] (const Eigen::VectorXf& state) {
    Eigen::VectorXf measurement(1);
    measurement(0) = state(0);
    return measurement;
  };

  ukf_ = std::make_unique<UKF>(model, kAlpha, kBeta, kKappa, kSensorNoise, kProcessNoise);
  ukf2_ = std::make_unique<UKF>(model, kAlpha, kBeta, kKappa, kSensorNoise, kProcessNoise);
  pid_ = std::make_unique<PID>();
  pid_->setGains(kGainP, kGainI, kGainD);
  pid_->setAntiwindup(false);
  pid_->setOutputLimits(-100.0, 100.0);
  pid_->setFilterCoeff(kPIDCoeffLowPass);
  pid2_ = std::make_unique<PID>();
  pid2_->setGains(kGainP, kGainI, kGainD);
  pid2_->setAntiwindup(false);
  pid2_->setOutputLimits(-100.0, 100.0);
  pid2_->setFilterCoeff(kPIDCoeffLowPass);
  attachInterrupt(digitalPinToInterrupt(kPinEnc1A), onClockLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kPinEnc2A), onClockRight, CHANGE);
  ibt2_->configPWM(kPwmResolution, kPwmFreq);
  ibt22_->configPWM(kPwmResolution, kPwmFreq);
  ibt2_->enable();
  ibt22_->enable();
  millisPrevLeft_ = millis();
  millisPrevRight_ = millisPrevLeft_;

  ibt22_->setSpeed(0.0f);
  ibt2_->setSpeed(0.0f);
  stepAngle_ = 2.0 * M_PI / kPulsePerRev;

  BaseType_t success = xTaskCreate(task,
                                  "task",
                                  4096,
                                  nullptr,
                                  4,
                                  &taskHandle_);
}


void loop() {
  vTaskDelete(NULL); // Delete main loop task
}

enum class TestType {
  WAVE,
  SQUARE
};


/// @brief Runs a motor test using a sine wave varying setpoint.
/// @param testType Type of test to run.
/// @param duration [s] Duration of the test.
/// @param scalarFrequency Oscitation frequency scalar
/// @param amplitude [tick/s] Wave: Amplitude of the sine wave. Square: 'On' amplitude.
/// @param offset [tick/s] Wave: Offset of the sine wave. Square: 'Off' amplitude.
/// @return [0] Error, [1] Error Time, [2] Error^2, [3] Error^2 Time, [4] Error^2 Time^2
std::array<float, 5> runWaveTest(const TestType testType, const float duration, const float scalarFrequency, const float amplitude, const float offset) {
  std::array<float, 5> results;
  std::vector<float> windowTime;
  windowTime.reserve(windowSize_);
  std::vector<float> windowError;
  windowError.reserve(windowSize_);
  std::vector<float> windowError2;
  windowError2.reserve(windowSize_);
  std::vector<float> windowErrorTime;
  windowErrorTime.reserve(windowSize_);
  std::vector<float> windowError2Time;
  windowError2Time.reserve(windowSize_);
  std::vector<float> windowError2Time2;
  windowError2Time2.reserve(windowSize_);
  std::vector<float> windowSetpoint;

  auto sineWave = [] (float amp, float freq, float offset) {
    return amp * sin(freq * static_cast<float>(millis()) / 1E3f) + offset;
  };

  auto squareWave = [] (float amp, float freq, float offset) {
    float trig = sin(freq * static_cast<float>(millis()) / 1E3f);

    if (trig < 0.0f) {  // Start off
      return amp;
    } 
    else {
      return offset;
    }
  };

  auto setPointFunc = testType == TestType::WAVE ? sineWave : squareWave;

  float startTime = static_cast<float>(millis()) / 1E3f;
  uint32_t now = millis();
  TickType_t lastWakeTime = xTaskGetTickCount();

  while ((static_cast<float>(now) / 1E3f) - startTime < duration) {
    xTaskDelayUntil(&lastWakeTime, loopDelayMs_);
    now = millis();
    float time = static_cast<float>(now) / 1E3f;
    float deltaTime = static_cast<float>(now - millisPrevLeft_) / 1E3f;
    millisPrevLeft_ = now;

    float currentLeft = lowPass_.filter(ina2192_->getCurrent());
    float timePoint = time - startTime;
    float setpoint = setPointFunc(amplitude, scalarFrequency, offset) / stepAngle_;

    int32_t rotationLeft = encoder_->getRotation();
    int32_t rotationLowPass = static_cast<int32_t>(round(lowPass2_.filter(rotationLeft)));
    ukf_->step(Eigen::VectorXf::Ones(1) * rotationLeft, Eigen::VectorXf::Ones(1) * currentLeft, loopDelayMs_);
    Eigen::VectorXf filtered = ukf_->getState();
    
    float powerLeft = pid_->calculateOutput(rotationLeft, setpoint, deltaTime);
    ibt2_->setSpeed(powerLeft);

    Serial.println(">setpoint: " + String(setpoint));
    Serial.println(">rotation: " + String(rotationLeft));
    Serial.println(">rotationLowPass: " + String(rotationLowPass));
    Serial.println(">rotationUKF: " + String(filtered(0)));
    Serial.println(">velocityUKF: " + String(filtered(1)));
    Serial.println(">power: " + String(currentLeft));

    windowTime.push_back(timePoint);
    float error = std::abs(rotationLeft - setpoint) * 1E-3f;  // Scaling error to prevent overflow
    windowError.push_back(error);
    windowErrorTime.push_back(error * timePoint);
    float errorSquared = error * error;
    windowError2.push_back(errorSquared);
    float errorSquaredTime = errorSquared * timePoint;
    windowError2Time.push_back(errorSquaredTime);
    windowError2Time2.push_back(errorSquaredTime * timePoint);
  }

  ibt2_->setSpeed(0.0f);
  results[0] = Integration::trapezoidal(windowError.begin(), windowError.end());
  results[1] = Integration::trapezoidal(windowErrorTime.begin(), windowErrorTime.end());
  results[2] = Integration::trapezoidal(windowError2.begin(), windowError2.end());
  results[3] = Integration::trapezoidal(windowError2Time.begin(), windowError2Time.end());
  results[4] = Integration::trapezoidal(windowError2Time2.begin(), windowError2Time2.end());

  return results;
}


void task(void *pvParameters) {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  const uint8_t indexResult = 4;
  const float freqScalar = 1.5f;
  const float amplitude = M_PI;
  const float offset = M_PI_2;
  // ku = 2.2
  // kp = 0.3
  std::vector<std::array<float, 3>> gains = {{0.44f, 2.933f, 0.0440f},
                                            {0.726f, 4.889f, 0.0733f}};
  std::vector<float> results;
  results.reserve(gains.size());

  for (size_t i = 0; i < gains.size(); ++i) {
    log_n("Test %i of %i", i + 1, gains.size());
    log_n("gains: P: %.2f, I: %.2f, D: %.2f", gains[i][0], gains[i][1], gains[i][2]);
    pid_->setGains(gains[i][0], gains[i][1], gains[i][2]);
    pid_->resetPID();
    results.push_back(runWaveTest(TestType::SQUARE, kTestDuration, freqScalar, amplitude, offset).at(indexResult));
    log_n("Test complete. Results:");
    log_n("Error: %.3f", results[0]);
    log_n("Error Time: %.3f", results[1]);
    log_n("Error^2: %.3f", results[2]);
    log_n("Error^2 Time: %.3f", results[3]);
    log_n("Error^2 Time^2: %.3f", results[4]);
    log_n("=====================================");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  ibt2_->setSpeed(0.0f);
  auto bestIt = std::min_element(results.begin(), results.end());
  uint8_t index = std::distance(results.begin(), bestIt);
  log_n("Best test was %i", index + 1);
  log_n("result: %.3f", *bestIt);

  for (;;) {
    vTaskDelay(60000 / portTICK_PERIOD_MS);

    /*
    
    */
  }
}