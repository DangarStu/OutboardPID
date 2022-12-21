#include "sensesp/transforms/curveinterpolator.h"

namespace sensesp {

class TemperatureInterpolator : public CurveInterpolator {
 public:

  TemperatureInterpolator(String config_path = "")
      : CurveInterpolator(NULL, config_path) {

    // Seed the CurveInterpolator with the resistances and
    // temperatures in celcius from the sensor datasheet:
    // http://pe-ltd.com/assets/coolant_temp.pdf
    
    float data_points[39][2] = {

      {-40, 102122}, {-35, 73340}, {-30, 53249}, {-25, 39064}, {-20, 28939},
      {-15, 21637}, {-10, 16321}, {-5, 12413}, {0, 9516}, {5, 7354}, {10, 5728},
      {15, 4496}, {20, 3555}, {25, 2830}, {30, 2268}, {35, 1828}, {40, 1483},
      {45, 1210}, {50, 992}, {55, 819}, {60, 679}, {65, 566}, {70, 475}, {75, 400},
      {80, 338}, {85, 287}, {90, 244.8}, {95, 209.7}, {100, 180.3}, {105, 155.6},
      {110, 134.7}, {115, 117.1}, {120, 102.2}, {125, 89.4}, {130, 78.5}, {135, 69.1}, 
      {140, 61.1}, {145, 54.1}, {150, 48.1}
    
    };

    clear_samples();

    // Load all the samples from the thermistor datasheet into the CurveInterpolator.
    for (int index=0; index < 39; index++) {
      add_sample(CurveInterpolator::Sample(data_points[index][1], data_points[index][0]));
    }
  }
};

}  // namespace sensesp