#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace irrometer {

const int MAX_WM_SENSORS = 8;

class Irrometer : public PollingComponent {
 public:
  // Use arrays to store sensor pointers
  Sensor *tension_sensors[MAX_WM_SENSORS]{nullptr};
  Sensor *temperature_sensors[MAX_WM_SENSORS]{nullptr};
  Sensor *resistance_sensors[MAX_WM_SENSORS]{nullptr};
  Sensor *resistance_a_sensors[MAX_WM_SENSORS]{nullptr};
  Sensor *resistance_b_sensors[MAX_WM_SENSORS]{nullptr};

  // Constructor - use default polling interval from YAML
  Irrometer() : PollingComponent() {}

  void setup() override;
  void update() override;
  void dump_config() override;

  // Setter methods to be called from generated code
  void set_wm_index(int index) { this->wm_indices_[index] = true; }
  void set_tension_sensor(int index, Sensor *sensor) { this->tension_sensors[index] = sensor; }
  void set_temperature_sensor(int index, Sensor *sensor) { this->temperature_sensors[index] = sensor; }
  void set_resistance_sensor(int index, Sensor *sensor) { this->resistance_sensors[index] = sensor; }
  void set_resistance_a_sensor(int index, Sensor *sensor) { this->resistance_a_sensors[index] = sensor; }
  void set_resistance_b_sensor(int index, Sensor *sensor) { this->resistance_b_sensors[index] = sensor; }

 private:
  double r1_resistance_a_;
  double r1_resistance_b_;
  double r1_resistance_;

  bool wm_indices_[MAX_WM_SENSORS]{false};

  // Hardcoded pins from original file. These could be made configurable in sensor.py
  const int d1_ = D1;
  const int d2_ = D2;
  const int d3_ = D8;
  const int d4_ = D5;
  const int d5_ = D6;
  const int d6_ = D7;
  const int a0_ = A0; // adc

  const int num_of_read_ = 3;
  const int rx_ = 10003;
  const long open_resistance_ = 35000;
  const long short_resistance_ = 200;
  const long short_cb_ = 240, open_cb_ = 255;
  const float c_factor_ = 1.1;
  float supply_v_ = 3.3;

  double sen_v_10k_1_ = 0, sen_v_10k_2_ = 0;

  int resistance_to_cb(double res, double tempc = 24.0, double cf = 1.0);
  double resistance_to_temp(double res);
  void read_resistance_with_polarity_swap(int input);
};

}  // namespace irrometer
}  // namespace esphome
