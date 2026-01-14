#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <soem/soem.h>
#include <string>
#include <thread>
#include <vector>

namespace EposConsts {
constexpr uint16_t CONTROL_WORD = 0x6040;
constexpr uint16_t STATUS_WORD = 0x6041;
constexpr uint16_t OP_MODE = 0x6060;
constexpr uint16_t TARGET_POS = 0x607A;
constexpr uint16_t ACTUAL_POS = 0x6064;
constexpr uint16_t ERROR_CODE = 0x603F;
constexpr uint16_t PROFILE_VEL = 0x6081;
constexpr uint16_t PROFILE_ACC = 0x6083;
constexpr uint16_t PROFILE_DEC = 0x6084;
constexpr int8_t MODE_PROFILE_POSITION = 1;
} // namespace EposConsts

// PDO STRUCTURES (Packed)
#pragma pack(push, 1)
struct PdoOutput {
  uint16_t control_word;
  int8_t operation_mode;
  int32_t target_position;
};

struct PdoInput {
  uint16_t status_word;
  int8_t mode_display;
  int32_t actual_position;
  uint16_t error_code;
};
#pragma pack(pop)

class Epos4Controller {
public:
  explicit Epos4Controller(const std::string &ifnet);

  ~Epos4Controller();

  // Public API

  void set_target_position(int motor_id, int32_t position_value);

  int32_t get_current_position(int motor_id);

  void set_profile_velocity(int motor_id, uint32_t velocity);

  int get_slave_count() const;

private:
  ecx_contextt ethercat_context;
  char io_map_buffer[4096];
  int connected_slave_count;

  std::thread worker;
  std::atomic<bool> is_driver_running;
  std::mutex ecat_mutex; // Protects EtherCAT bus access

  struct SharedMotorData {
    std::atomic<int32_t> new_target_position{0};
    std::atomic<bool> has_new_command{false};
    std::atomic<int32_t> position{0};

    // Pointers to SOEM memory (Observers)
    PdoOutput *pdo_output = nullptr;
    PdoInput *pdo_input = nullptr;
  };
  std::vector<std::unique_ptr<SharedMotorData>> motor_data_list;

  void worker_loop();
  void configure_slave(int slave_index);
  void configure_pdo(int slave_index);

  template <typename DataType>
  bool send_sdo_write(uint16_t slave_index, uint16_t index, uint8_t sub_index,
                      DataType value);
};
