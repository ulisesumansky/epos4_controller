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
#include <stdexcept>

class Epos4Controller {
  public:
    explicit Epos4Controller(const std::string &ifnet);
    ~Epos4Controller();

    // Public API
    void set_target_position(int motor_id, int32_t position_value);
    int32_t get_current_position(int motor_id);
    void set_profile_velocity(int motor_id, uint32_t velocity);
    int get_slave_count() const;

    template <typename DataType>
    bool sdo_write(uint16_t slave_index, uint16_t index, uint8_t sub_index, DataType value) {
        std::lock_guard<std::mutex> lock(ecat_mutex);
        return _sdo_write(slave_index, index, sub_index, value);
    }

    template <typename DataType>
    DataType sdo_read(uint16_t slave_index, uint16_t index, uint8_t sub_index) {
        std::lock_guard<std::mutex> lock(ecat_mutex);
        return _sdo_read<DataType>(slave_index, index, sub_index);
    }

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

    void _worker_loop();
    void _configure_slaves_in_network();
    int _set_operational_state();
    void _set_motor_shared_data(int slave_index);
    void _set_slave_default_values(int slave_index);
    void _configure_pdo(int slave_index);
    void _state_machine(SharedMotorData &motor, uint16_t status);
    void _start_movement(SharedMotorData &motor, uint16_t status);
    void _wait_for_motors_enabled();
    template <typename DataType>
    bool _sdo_write(uint16_t slave_index, uint16_t index, uint8_t sub_index, DataType value) {
        int wkc = ecx_SDOwrite(&ethercat_context, slave_index, index, sub_index, FALSE,
                               sizeof(value), &value, EC_TIMEOUTRXM);
        return (wkc > 0);
    }

    template <typename DataType>
    DataType _sdo_read(uint16_t slave_index, uint16_t index, uint8_t sub_index) {
        DataType value = {};
        int size = sizeof(DataType);
        int wkc = ecx_SDOread(&ethercat_context, slave_index, index, sub_index, FALSE, &size,
                              &value, EC_TIMEOUTRXM);

        if (wkc <= 0) {
            throw std::runtime_error("Failed to read SDO index: " + std::to_string(index));
        }
        return value;
    }
};
