#pragma once

#include "epos4_definitions.hpp"
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <soem/soem.h>
#include <string>
#include <thread>
#include <vector>

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
  std::mutex ecat_mutex;

  struct SharedMotorData {
    std::atomic<int32_t> new_target_position{0};
    std::atomic<bool> has_new_command{false};
    std::atomic<int32_t> position{0};

    // Pointers to SOEM mapped memory
    PdoOutput *pdo_output = nullptr;
    PdoInput *pdo_input = nullptr;
  };

  std::vector<std::unique_ptr<SharedMotorData>> motor_data_list;

  void worker_loop();
  void configure_slaves_in_network(ecx_contextt &ec_ctx);
  int set_operational_state(ecx_contextt &ec_ctx);
  void set_motor_shared_data(ecx_contextt &ec_ctx, int slave_index);
  void set_slave_default_values(int slave_index);
  void configure_pdo(int slave_index);
  void state_machine(SharedMotorData &motor, uint16_t status);
  void start_movement(SharedMotorData &motor, uint16_t status);

  template <typename DataType>
  bool send_sdo_write(uint16_t slave_index, uint16_t index, uint8_t sub_index,
                      DataType value);
};
