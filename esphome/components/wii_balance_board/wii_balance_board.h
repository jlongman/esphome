#pragma once

#include "esphome/core/component.h"
#include "esphome/components/button/button.h"
#include "esphome/components/sensor/sensor.h"
#include "wii.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/core/preferences.h"

#include "task_queue.h"
#include <unordered_map>

namespace esphome {
namespace wii_balance_board {

struct PairedBoardPreference {
  uint32_t magic{0};
  uint64_t bdaddr{0};
  bool hasLinkKey{false};
  uint8_t linkKeyData[16]{};
};

struct Sample {
  float samples[64] = {NAN};
  size_t sample_count{0};
  uint8_t battery{0};
  uint8_t temperature{0};
  uint8_t referenceTemperature{0};
  float measurement{NAN};
};

class WiiBalanceBoard : public Component {
 public:
  WiiBalanceBoard();

  void setup() override;
  void loop() override;
  float get_setup_priority() const override;
  void dump_config() override;
  void sync(bool enable);

  void set_temperature_sensor(sensor::Sensor *temperature_sensor);
  void set_reference_temperature_sensor(sensor::Sensor *reference_temperature_sensor);
  void set_battery_level(sensor::Sensor *battery_level);
  void set_weight(sensor::Sensor *weight);
  void set_syncing(binary_sensor::BinarySensor *syncing);
  void set_stddev(float stddev);
  void set_led_pin(int led_pin);

 protected:
  void board_connected(uint16_t handle);
  void board_disconnected(uint16_t handle);
  void board_paired(uint64_t bdaddr, bool has_link_key, const uint8_t *link_key_data);
  void board_sample(uint16_t handle, uint8_t battery, uint8_t reference_temp, uint8_t temperature, float topRightLoad,
                    float bottomRightLoad, float topLeftLoad, float bottomLeftLoad);

  detail::Bluetooth bluetooth;
  detail::Wii wii;
  std::unordered_map<uint16_t, Sample> sampleMap;
  detail::TaskQueue queue;
  ESPPreferenceObject paired_board_pref_;

  float std_dev_;
  int led_pin_;
  bool bluetooth_ready_{false};
  bool sync_on_ready_{false};

  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *reference_temperature_sensor_{nullptr};
  sensor::Sensor *battery_level_{nullptr};
  sensor::Sensor *weight_{nullptr};
  binary_sensor::BinarySensor *syncing_{nullptr};
};

}  // namespace wii_balance_board
}  // namespace esphome
