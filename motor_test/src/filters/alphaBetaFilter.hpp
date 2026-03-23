#ifndef ALPHABETAFILTER_HPP
#define ALPHABETAFILTER_HPP

/*
  Alpha-beta filter. Good for estimating velocity and acceleration to filter encoder noise.
*/



struct EncoderData {
  float rotation = 0.0;      //!< [rad] Angular rotation.
  float velocity = 0.0;      //!< [rad/s] Angular velocity.
  float acceleration = 0.0;  //!< [rad/s^2] Angular acceleration.

  EncoderData(const float rotationIn = 0, 
              const float velocityIn = 0,
              const float accelerationIn = 0) : 
                  rotation(rotationIn), 
                  velocity(velocityIn),
                  acceleration(accelerationIn) {};

  bool operator==(const EncoderData& other) 
  {
    return (rotation == other.rotation) && (velocity == other.velocity) && (acceleration == other.acceleration);
  }


  bool operator!=(const EncoderData& other) 
  {
    return !(*this == other);
  }


  EncoderData& operator=(const EncoderData& other) 
  {
    rotation = other.rotation;
    velocity = other.velocity;
    acceleration = other.acceleration;
    return *this;
  }
};


class AlphaBetaFilter {
public:
  /// @brief Makes a new alpha-beta filter.
  /// @param alpha [0-1] Controls positional filtering. usually 0.5 to 0.85. Lower increases smoothing.
  /// @param beta [0-2] Controls velocity filtering. usually 0.1 to 0.3. Lower increases smoothing.
  AlphaBetaFilter(float alpha = 0.5, 
                  float beta = 0.4);
  

  /// @brief Takes the input value and outputs the filtered result.
  /// @param newValue New input value.
  /// @param deltaTime [ms] Time since last update.
  /// @return Filtered value.
  EncoderData filter(float newValue, float deltaTime);

protected:
  bool initialized_;
  const float alpha_;
  const float beta_;
  float xHat_;
  float yHat_;
};


class AlphaBetaGammaFilter : public AlphaBetaFilter {
public:
  /// @brief Makes a new alpha-beta filter.
  /// @param alpha [0-1] Controls positional filtering. usually 0.5 to 0.85. Lower increases x smoothing.
  /// @param beta [0-2] Controls velocity filtering. usually 0.1 to 0.3. Lower increases y smoothing.
  /// @param gamma [0-2] Controls acceleration filtering. usually 0.1 to 0.3. Lower increases z smoothing.
  AlphaBetaGammaFilter(float alpha = 0.5, 
                       float beta = 0.4, 
                       float gamma = 0.1);

  EncoderData filter(float newValue, float deltaTime);

private:
  const float gamma_;
  float zHat_;
};

#endif // ALPHABETAFILTER_HPP
