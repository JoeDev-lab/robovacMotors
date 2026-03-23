#ifndef INTEGRATION_HPP
#define INTEGRATION_HPP

#include <Arduino.h>
#include <cstdint>
#include <vector>


enum class IntegrationMethod {
  RECTANGULAR_LEFT,
  RECTANGULAR_RIGHT,
  RECTANGULAR_CENTER
};


/// @brief Collection of integration methods. All methods static.
class Integration {
public:
  Integration(void) {};


  /// @brief Calculates the integral using the rectangular rule.
  /// @param method Integration method. Must be RECTANGULAR_LEFT, RECTANGULAR_RIGHT, or RECTANGULAR_CENTER.
  /// @param valuesX Vector of x values.
  /// @param valuesY Vector of y values.
  /// @return Integral of y(x).
  template <typename IterX, typename IterY>
  static float rectangularRule(const IntegrationMethod method,  
                              IterY yBegin, IterY yEnd,
                              IterX xBegin, IterX xEnd) {
      
    size_t sizeX = std::distance(xBegin, xEnd);
    size_t sizeY = std::distance(yBegin, yEnd);

    if (sizeX < 2 || sizeX != sizeY) {
      Serial.println("Error: Size mismatch or insufficient points.");
      return 0.0f;
    }

    float sum = 0.0f;
    auto xIt = xBegin;
    auto yIt = yBegin;
    auto xEndMinusOne = std::prev(xEnd);

    while (xIt != xEndMinusOne) {
      float dx = *std::next(xIt) - *xIt; // The width of the current interval

      switch (method) {
        case IntegrationMethod::RECTANGULAR_LEFT:
          sum += (*yIt) * dx;
          break;

        case IntegrationMethod::RECTANGULAR_RIGHT:
          sum += (*std::next(yIt)) * dx;
          break;

        case IntegrationMethod::RECTANGULAR_CENTER:
          sum += ((*yIt + *std::next(yIt)) / 2.0f) * dx;
          break;
      }

      ++xIt;
      ++yIt;
    }

    return sum;
  }


  /// @brief Calculates the integral using the trapezoidal rule.
  /// @param begin Container start iterator
  /// @param end Container end iterator
  /// @return Integral of y(x).
  template <typename Iterator>
  static float trapezoidal(Iterator begin, Iterator end) {
    if (begin == end) return 0.0f;
    float sum = *begin + *end;
    size_t count = 0;

    for (auto it = begin; it != end; ++it) {
      sum += 2 * *it;
      count++;
    }

    return sum *= count * 0.5f;
  }


  /// @brief Calculates the integral using Simpson's 1/3 and 3/8 rules.
  /// @param begin Container start iterator
  /// @param end Container end iterator
  /// @param intervalSize Duration between points.
  /// @return Integral of y(x).
  template <typename Iterator>
  static float simpsonsMixed(Iterator begin, Iterator end, const uint32_t intervalSize) {
    if (begin == end) return 0.0f;
    size_t size = std::distance(begin, end);
    float integral = 0.0f;
    Iterator it = begin;

    // We process the bulk of the data using Simpson's 1/3 rule (needs 3 points/2 intervals)
    // We stop when we have 4 or fewer points left so we can handle the "tail" specially.
    while (std::distance(it, end) > 4) {
        integral += simpsonsOneThird(intervalSize, *it, *(it + 1), *(it + 2));
        it += 2; // Move 2 intervals (3 points)
    }

    // Handle the remaining points
    size_t remaining_points = std::distance(it, end);

    // Simpson's 3/8 rule uses 4 points
    if (remaining_points == 4) {
        integral += simpsonsThreeEighths(intervalSize, *it, *(it + 1), *(it + 2), *(it + 3));
    } // Simpson's 1/3 rule uses 3 points
    else if (remaining_points == 3) {
        integral += simpsonsOneThird(intervalSize, *it, *(it + 1), *(it + 2));
    }  // Trapezoidal rule for the last 2 points
    else if (remaining_points == 2) {
        integral += trapezoidalOneHalf(intervalSize, *it, *(it + 1));
    }

    return integral;
  }


  /// @brief Calculates the integral using Simpson's 1/3 and 3/8 rules.
  /// @param dataBegin Data container start iterator
  /// @param dataEnd Data container end iterator
  /// @param timeBegin Data interval start iterator
  /// @param timeEnd Data interval end iterator
  /// @return Integral of y(x).
  template <typename DataIter, typename TimeIter>
  static float simpsonsMixed(DataIter dataBegin, DataIter dataEnd, TimeIter timeBegin, TimeIter timeEnd) {
    size_t size = std::distance(dataBegin, dataEnd);
    size_t timeSize = std::distance(timeBegin, timeEnd);

    if (size < 2 || size != timeSize) {
      return 0.0f; // Data and Time must match
    }

    // Calculate DeltaX (Average interval in seconds)
    // We use the first and last timestamps in the window
    float totalDeltaTime = static_cast<float>(*std::prev(timeEnd) - *timeBegin) / 1000.0f;
    float deltaX = totalDeltaTime / static_cast<float>(size - 1);

    // Standard Simpson's Logic
    float integral = 0.0f;
    DataIter it = dataBegin;

    while (std::distance(it, dataEnd) > 4) {
      integral += simpsonsOneThird(deltaX, *it, *(it + 1), *(it + 2));
      it += 2;
    }

    size_t remaining = std::distance(it, dataEnd);
    if (remaining == 4) {
      integral += simpsonsThreeEighths(deltaX, *it, *(it + 1), *(it + 2), *(it + 3));
    } else if (remaining == 3) {
      integral += simpsonsOneThird(deltaX, *it, *(it + 1), *(it + 2));
    } else if (remaining == 2) {
      integral += trapezoidalOneHalf(deltaX, *it, *(it + 1));
    }

    return integral;
  }


private:
  static float simpsonsOneThird(float h, float f0, float f1, float f2) {
    return (h / 3.0) * (f0 + (4.0 * (f1 + f2)));
  }

  static float simpsonsThreeEighths(float h, float f0, float f1, float f2, float f3) {
    return (3.0 * h / 8.0) * (f0 + (3.0 * (f1 + f2)) + f3);
  }

  static float trapezoidalOneHalf(float h, float f0, float f1) {
    return (h / 2.0) * (f0 + f1);
  }

  template <typename Iterator>
  static uint32_t calculateIntervalSize(Iterator begin, Iterator end, const size_t valueCount) {
    return round(*end - *begin / (valueCount - 1));
  }

};

#endif // INTEGRATION_HPP
