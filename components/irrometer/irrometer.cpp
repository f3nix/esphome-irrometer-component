#include "irrometer.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace irrometer {

static const char *const TAG = "irrometer";

void Irrometer::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Irrometer component...");
  pinMode(this->d1_, OUTPUT);
  pinMode(this->d2_, OUTPUT);
  pinMode(this->d3_, OUTPUT);
  pinMode(this->d4_, OUTPUT);
  pinMode(this->d5_, OUTPUT);
  pinMode(this->d6_, OUTPUT);
  delay(100);

  // enable the MUX
  digitalWrite(this->d4_, LOW);
  // address the MUX (channel 0)
  digitalWrite(this->d1_, LOW);
  digitalWrite(this->d2_, LOW);
  digitalWrite(this->d3_, LOW);
  delay(100);

  // set both excitation pins low
  digitalWrite(this->d5_, LOW);
  digitalWrite(this->d6_, LOW);
  delay(100);
}

void Irrometer::dump_config() {
  ESP_LOGCONFIG(TAG, "Irrometer Component:");
  LOG_UPDATE_INTERVAL(this);
  for (int i = 0; i < MAX_WM_SENSORS; ++i) {
    if (this->wm_indices_[i]) {
      ESP_LOGCONFIG(TAG, "  Watermark Sensor %d:", i + 1);
      LOG_SENSOR("    ", "Soil Water Tension", this->tension_sensors[i]);
      LOG_SENSOR("    ", "Soil Temperature", this->temperature_sensors[i]);
      LOG_SENSOR("    ", "Resistance", this->resistance_sensors[i]);
      LOG_SENSOR("    ", "Resistance A", this->resistance_a_sensors[i]);
      LOG_SENSOR("    ", "Resistance B", this->resistance_b_sensors[i]);
    }
  }
}

void Irrometer::update() {
  ESP_LOGD(TAG, "Updating Irrometer sensors...");
  double temp = 24.0; // Default temperature

  for (int i = 0; i < MAX_WM_SENSORS; i++) {
    // Only read and publish if a sensor for this index is configured
    if (this->wm_indices_[i]) {
      ESP_LOGD(TAG, "Reading from MUX channel %d", i);
      this->read_resistance_with_polarity_swap(i);

      if (this->resistance_a_sensors[i] != nullptr) {
        this->resistance_a_sensors[i]->publish_state(this->r1_resistance_a_);
      }
      if (this->resistance_b_sensors[i] != nullptr) {
        this->resistance_b_sensors[i]->publish_state(this->r1_resistance_b_);
      }
      if (this->resistance_sensors[i] != nullptr) {
        this->resistance_sensors[i]->publish_state(this->r1_resistance_);
      }

      // Check if this sensor is a temperature or tension sensor based on common practice (odd/even)
      // This logic is based on your original update() function
      if (i % 2 == 0) { // Even channels (0, 2, 4, 6) are temperature
        temp = this->resistance_to_temp(this->r1_resistance_);
        if (this->temperature_sensors[i] != nullptr) {
          this->temperature_sensors[i]->publish_state(temp);
        }
      } else { // Odd channels (1, 3, 5, 7) are tension
        if (this->tension_sensors[i] != nullptr) {
          int cb = this->resistance_to_cb(this->r1_resistance_, temp);
          this->tension_sensors[i]->publish_state(cb);
        }
      }
    }
  }
}

void Irrometer::read_resistance_with_polarity_swap(int input) {
  double aread_a1 = 0;
  double aread_a2 = 0;

  for (int i = 0; i < this->num_of_read_; i++) {
    digitalWrite(this->d4_, LOW); // enable the MUX
    digitalWrite(this->d1_, (input & 1));
    digitalWrite(this->d2_, ((input >> 1) & 1));
    digitalWrite(this->d3_, ((input >> 2) & 1));
    delay(100);

    digitalWrite(this->d6_, LOW);
    digitalWrite(this->d5_, HIGH);
    delayMicroseconds(90);
    aread_a1 += analogRead(this->a0_);
    digitalWrite(this->d5_, LOW);

    delay(100);

    digitalWrite(this->d6_, HIGH);
    delayMicroseconds(90);
    aread_a2 += analogRead(this->a0_);
    digitalWrite(this->d6_, LOW);
    digitalWrite(this->d4_, HIGH); // disable the MUX
  }
  delay(100);

  this->sen_v_10k_1_ = ((aread_a1 / 1024.0) * this->supply_v_) / this->num_of_read_;
  this->sen_v_10k_2_ = ((aread_a2 / 1024.0) * this->supply_v_) / this->num_of_read_;

  this->r1_resistance_a_ = (this->rx_ * (this->supply_v_ - this->sen_v_10k_1_) / this->sen_v_10k_1_);
    if (r1_resistance_a_ < 1400) { r1_resistance_a_ += 580; }
    else if (r1_resistance_a_ < 2340) { r1_resistance_a_ += 597; }
    else if (r1_resistance_a_ < 3300) { r1_resistance_a_ += 648; }
    else if (r1_resistance_a_ < 4250) { r1_resistance_a_ += 680; }
    else if (r1_resistance_a_ < 9000) { r1_resistance_a_ += 740; }
    else if (r1_resistance_a_ < 20000) { r1_resistance_a_ += 964; }

  this->r1_resistance_b_ = this->rx_ * this->sen_v_10k_2_ / (this->supply_v_ - this->sen_v_10k_2_);
    if (r1_resistance_b_ < 2400) { r1_resistance_b_ -= 277; }
    else if (r1_resistance_b_ < 3500) { r1_resistance_b_ -= 412; }
    else if (r1_resistance_b_ < 4600) { r1_resistance_b_ -= 510; }
    else if (r1_resistance_b_ < 5700) { r1_resistance_b_ -= 629; }
    else if (r1_resistance_b_ < 11450) { r1_resistance_b_ -= 731; }
    else if (r1_resistance_b_ < 20000) { r1_resistance_b_ -= 1470; }

  this->r1_resistance_ = (this->r1_resistance_a_ + this->r1_resistance_b_) / 2.0;
}

int Irrometer::resistance_to_cb(double res, double tempc, double cf) {
    int WM_CB;
    double resK = res / 1000.0;
    float tempD = 1.00 + 0.018 * (tempc - 24.00);

    if (res > 550.00) {
        if (res > 8000.00) {
            WM_CB = (-2.246 - 5.239 * resK * tempD - 0.06756 * resK * resK * tempD * tempD) * cf;
        } else if (res > 1000.00) {
            WM_CB = ((-3.213 * resK - 4.093) / (1 - 0.009733 * resK - 0.01205 * tempc)) * cf;
        } else {
            WM_CB = ((resK * 23.156 - 12.736) * tempD) * cf;
        }
    } else {
        if (res > 300.00) { WM_CB = 0.00; }
        if (res < 300.00 && res >= short_resistance_) { WM_CB = short_cb_; }
    }
    if (res >= open_resistance_ || res == 0) { WM_CB = open_cb_; }

    return abs(WM_CB);
}

double Irrometer::resistance_to_temp(double res) {
    double TempC = (-23.89 * (log(res))) + 246.00;
    if (res < 0 || res > 30000 ) { TempC = 24.0; }
    return TempC;
}

}  // namespace irrometer
}  // namespace esphome
