#include "irrometer.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace irrometer {

using namespace esphome::sensor;

static const char *const TAG = "irrometer";

void Irrometer::setup() {
    ESP_LOGCONFIG(TAG, "Setting up Irrometer component...");
    pinMode(this->d1_, OUTPUT);
    pinMode(this->d2_, OUTPUT);
    pinMode(this->d3_, OUTPUT);
    pinMode(this->d4_, OUTPUT);
    pinMode(this->d5_, OUTPUT);
    pinMode(this->d6_, OUTPUT);

    // Initial state: MUX disabled, excitation low
    digitalWrite(this->d4_, HIGH); // Disable MUX initially
    digitalWrite(this->d5_, LOW);
    digitalWrite(this->d6_, LOW);
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
    if (this->is_measuring_) {
        ESP_LOGW(TAG, "Skipping update, previous measurement still in progress");
        return;
    }
    ESP_LOGD(TAG, "Starting Irrometer measurement sequence...");
    this->is_measuring_ = true;
    this->current_sensor_index_ = 0;
    this->last_temperature_ = 24.0;
    this->start_next_sensor();
}

void Irrometer::start_next_sensor() {
    // Find next active sensor
    while (this->current_sensor_index_ < MAX_WM_SENSORS && !this->wm_indices_[this->current_sensor_index_]) {
        this->current_sensor_index_++;
    }

    // If we processed all sensors, finish
    if (this->current_sensor_index_ >= MAX_WM_SENSORS) {
        ESP_LOGD(TAG, "All sensors read. Sequence complete.");
        this->is_measuring_ = false;
        // Ensure all pins are safe
        digitalWrite(this->d4_, HIGH); // Disable MUX
        digitalWrite(this->d5_, LOW);
        digitalWrite(this->d6_, LOW);
        return;
    }

    // Initialize for this sensor
    this->current_read_iteration_ = 0;
    this->acc_a1_ = 0;
    this->acc_a2_ = 0;

    // Start the step chain
    this->step_enable_mux();
}

// STEP 1: Enable MUX and wait
void Irrometer::step_enable_mux() {
    int input = this->current_sensor_index_;

    digitalWrite(this->d4_, LOW); // Enable MUX
    digitalWrite(this->d1_, (input & 1));
    digitalWrite(this->d2_, ((input >> 1) & 1));
    digitalWrite(this->d3_, ((input >> 2) & 1));

    // Non-blocking wait: 100ms
    this->set_timeout(100, [this]() { this->step_read_polarity_1(); });
}

// STEP 2: Read Polarity 1
void Irrometer::step_read_polarity_1() {
    digitalWrite(this->d6_, LOW);
    digitalWrite(this->d5_, HIGH);
    delayMicroseconds(90); // Short delay is fine to block
    this->acc_a1_ += analogRead(this->a0_);
    digitalWrite(this->d5_, LOW);

    // Non-blocking wait: 100ms
    this->set_timeout(100, [this]() { this->step_read_polarity_2(); });
}

// STEP 3: Read Polarity 2
void Irrometer::step_read_polarity_2() {
    digitalWrite(this->d6_, HIGH);
    delayMicroseconds(90);
    this->acc_a2_ += analogRead(this->a0_);
    digitalWrite(this->d6_, LOW);
    digitalWrite(this->d4_, HIGH); // Disable MUX briefly (per original logic)

    this->current_read_iteration_++;

    if (this->current_read_iteration_ < this->num_of_read_) {
        // Loop back to Step 1 (Enable MUX)
        // Original code had no specific delay here, but we proceed immediately to enable mux
        this->step_enable_mux();
    } else {
        // Loop finished. Wait final 100ms before calculating
        this->set_timeout(100, [this]() { this->step_finalize_sensor(); });
    }
}

// STEP 4: Calculate and Publish
void Irrometer::step_finalize_sensor() {
    int i = this->current_sensor_index_;

    this->sen_v_10k_1_ = ((this->acc_a1_ / 1024.0) * this->supply_v_) / this->num_of_read_;
    this->sen_v_10k_2_ = ((this->acc_a2_ / 1024.0) * this->supply_v_) / this->num_of_read_;

    // Resistance Calculation A
    this->r1_resistance_a_ = (this->rx_ * (this->supply_v_ - this->sen_v_10k_1_) / this->sen_v_10k_1_);
    if (r1_resistance_a_ < 1400) { r1_resistance_a_ += 580; }
    else if (r1_resistance_a_ < 2340) { r1_resistance_a_ += 597; }
    else if (r1_resistance_a_ < 3300) { r1_resistance_a_ += 648; }
    else if (r1_resistance_a_ < 4250) { r1_resistance_a_ += 680; }
    else if (r1_resistance_a_ < 9000) { r1_resistance_a_ += 740; }
    else if (r1_resistance_a_ < 20000) { r1_resistance_a_ += 964; }

    // Resistance Calculation B
    this->r1_resistance_b_ = this->rx_ * this->sen_v_10k_2_ / (this->supply_v_ - this->sen_v_10k_2_);
    if (r1_resistance_b_ < 2400) { r1_resistance_b_ -= 277; }
    else if (r1_resistance_b_ < 3500) { r1_resistance_b_ -= 412; }
    else if (r1_resistance_b_ < 4600) { r1_resistance_b_ -= 510; }
    else if (r1_resistance_b_ < 5700) { r1_resistance_b_ -= 629; }
    else if (r1_resistance_b_ < 11450) { r1_resistance_b_ -= 731; }
    else if (r1_resistance_b_ < 20000) { r1_resistance_b_ -= 1470; }

    this->r1_resistance_ = (this->r1_resistance_a_ + this->r1_resistance_b_) / 2.0;

    const float eps = 0.001f;
    bool invalid_resistance = (sen_v_10k_1_ <= eps) ||
                              ((supply_v_ - sen_v_10k_2_) <= eps) ||
                              !std::isfinite(this->r1_resistance_a_) ||
                              !std::isfinite(this->r1_resistance_b_) ||
                              !std::isfinite(this->r1_resistance_);;
    if (invalid_resistance) {
        if (this->resistance_a_sensors[i] != nullptr) this->resistance_a_sensors[i]->publish_state(NAN);
        if (this->resistance_b_sensors[i] != nullptr) this->resistance_b_sensors[i]->publish_state(NAN);
        if (this->resistance_sensors[i] != nullptr) this->resistance_sensors[i]->publish_state(NAN);
    } else {
        // Publish raw resistances
        if (this->resistance_a_sensors[i] != nullptr) this->resistance_a_sensors[i]->publish_state(this->r1_resistance_a_);
        if (this->resistance_b_sensors[i] != nullptr) this->resistance_b_sensors[i]->publish_state(this->r1_resistance_b_);
        if (this->resistance_sensors[i] != nullptr) this->resistance_sensors[i]->publish_state(this->r1_resistance_);
    }

    // Publish Temp or Tension
    if (i % 2 == 0) {
        // This is a TEMPERATURE sensor.
        // Calculate and SAVE the temperature for the next guy to use.
        this->last_temperature_ = this->resistance_to_temp(this->r1_resistance_);

        if (this->temperature_sensors[i] != nullptr) {
            this->temperature_sensors[i]->publish_state(this->last_temperature_);
        }
    } else {
        // This is a TENSION sensor.
        // Use the stored `last_temperature_` from the previous step.
        if (this->tension_sensors[i] != nullptr) {
            int cb = this->resistance_to_cb(this->r1_resistance_, this->last_temperature_);
            this->tension_sensors[i]->publish_state(cb);
        }
    }

    // Move to next sensor
    this->current_sensor_index_++;
    this->start_next_sensor();
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
        else if (res >= short_resistance_) { WM_CB = short_cb_; }
        else { WM_CB = open_cb_; }
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
