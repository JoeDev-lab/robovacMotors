#ifndef IBT2_HPP
#define IBT2_HPP

#include <cstdint>


class IBT2 {
public:
  IBT2(const uint8_t pinFwd, 
       const uint8_t pinBck, 
       const uint8_t pinEnFwd, 
       const uint8_t pinEnBck);

  
  void configPWM(const uint8_t pwmResolutionBits, 
                const uint32_t pwmFreq);

  void enable(void);


  void disable(void);

  /**
   * @brief Set the driver speed.
   * 
   * @param speed [% as -100 to 100] Desired speed.
   */
  void setSpeed(float speed);
  
  
private:
  const uint8_t kPinFwd = 0;        //!< Forward PWM pin.
  const uint8_t kPinBck = 0;        //!< Backward PWM pin.
  const uint8_t kPinEnFwd = 0;      //!< Forward enable pin.
  const uint8_t kPinEnBck = 0;      //!< Backward enable pin.
  
  float resolutionSpeed_ = 0; //!< PWM output step resolution.
  uint16_t pwmMax_ = 0;       //!< PWM max value.
  float setpointSpeed_ = 0;
};

#endif // IBT2_HPP
