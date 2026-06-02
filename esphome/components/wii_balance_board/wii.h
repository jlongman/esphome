#pragma once

#include <array>
#include "bluetooth.h"
#include <unordered_map>
#include <memory>
#include <optional>

namespace esphome::wii_balance_board::detail {

struct BalanceBoardConnected {
  uint16_t handle;
};

struct BalanceBoardDisconnected {
  uint16_t handle;
};

struct BalanceBoardData {
  uint16_t handle;
  uint16_t tr;
  uint16_t br;
  uint16_t tl;
  uint16_t bl;
  uint8_t referenceTemperature;
  uint8_t temperature;
  uint8_t batteryLevel;
};

struct ScanStarted {};

struct ScanStopped {};

struct BalanceBoardPaired {
  uint64_t bdaddr;
  bool hasLinkKey;
  uint8_t linkKeyData[16];
};

using WiiEvent = std::variant<BalanceBoardConnected, BalanceBoardDisconnected, BalanceBoardData, BalanceBoardPaired,
                              ScanStarted, ScanStopped>;

class Wii {
  struct BalanceBoard;
  Bluetooth *bluetooth;
  std::unordered_map<uint16_t, std::unique_ptr<BalanceBoard>> connectedBoards;
  std::function<void(const WiiEvent &)> eventListener;

 public:
  Wii(Bluetooth *bluetooth);
  ~Wii();
  Wii(const Wii &) = delete;
  Wii &operator=(const Wii &) = delete;

  void onEvent(std::function<void(const WiiEvent &)> eventListener);
  void sync(bool enable);
  void step();

  void disconnect(uint16_t handle, uint16_t psm);

  void set_paired_board(uint64_t bdaddr);
  std::optional<uint64_t> paired_board() const;
  void set_link_key(uint64_t bdaddr, const uint8_t *linkKeyData);
  bool get_link_key(uint64_t bdaddr, uint8_t *linkKeyData) const;

 private:
  struct PairedBoard {
    uint64_t bdaddr{0};
    bool hasLinkKey{false};
    std::array<uint8_t, 16> linkKey{};
  };

  std::optional<PairedBoard> pairedBoard_;
};

}  // namespace esphome::wii_balance_board::detail
