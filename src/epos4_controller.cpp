#include "epos4_controller.hpp"
#include <chrono>
#include <iostream>

using namespace EposConsts;

Epos4Controller::Epos4Controller(const std::string &network_interface_name) {
    std::cout << "[EposDriver] Initializing on " << network_interface_name << "..." << std::endl;

    if (ecx_init(&ethercat_context, network_interface_name.c_str()) <= 0) {
        throw std::runtime_error("Failed to init SOEM. Check interface name and root permissions.");
    }

    if (ecx_config_init(&ethercat_context) <= 0) {
        throw std::runtime_error("No slaves found on bus.");
    }

    _configure_slaves_in_network();
    ecx_config_map_group(&ethercat_context, io_map_buffer, 0);
    ecx_configdc(&ethercat_context);

    for (int slave_index = 1; slave_index <= connected_slave_count; slave_index++) {
        _set_motor_shared_data(slave_index);
  }

    if (!_set_operational_state()) {
        is_driver_running = true;
        worker = std::thread(&Epos4Controller::_worker_loop, this);
        _wait_for_motors_enabled();
        std::cout << "[EposDriver] System Running." << std::endl;
    } else {
        throw std::runtime_error("Could not switch to Operational State.");
    }
}

Epos4Controller::~Epos4Controller() {
    is_driver_running = false;
    if (worker.joinable()) {
        worker.join();
    }
    ecx_close(&ethercat_context);
    std::cout << "[EposDriver] Closed." << std::endl;
}

void Epos4Controller::_configure_pdo(int slave_index) {
    // Map RxPDO (Outputs: Master -> Slave)
    // Clear Mapping (Subindex 0 = 0)
    _sdo_write(slave_index, RX_PDO_MAP_INDEX, 0x00, (uint8_t)0);

    // Map Objects
    _sdo_write(slave_index, RX_PDO_MAP_INDEX, 0x01, MAP_OBJ_CONTROL_WORD);
    _sdo_write(slave_index, RX_PDO_MAP_INDEX, 0x02, MAP_OBJ_OP_MODE);
    _sdo_write(slave_index, RX_PDO_MAP_INDEX, 0x03, MAP_OBJ_TARGET_POS);

    //  Enable Mapping (Subindex 0 = Number of mapped objects)
    _sdo_write(slave_index, RX_PDO_MAP_INDEX, 0x00, (uint8_t)3);

    // Map TxPDO (Inputs: Slave -> Master)
    // Clear Mapping
    _sdo_write(slave_index, TX_PDO_MAP_INDEX, 0x00, (uint8_t)0);

    // Map Objects
    _sdo_write(slave_index, TX_PDO_MAP_INDEX, 0x01, MAP_OBJ_STATUS_WORD);
    _sdo_write(slave_index, TX_PDO_MAP_INDEX, 0x02, MAP_OBJ_MODE_DISP);
    _sdo_write(slave_index, TX_PDO_MAP_INDEX, 0x03, MAP_OBJ_ACTUAL_POS);
    _sdo_write(slave_index, TX_PDO_MAP_INDEX, 0x04, MAP_OBJ_ERROR_CODE);

    // Enable Mapping
    _sdo_write(slave_index, TX_PDO_MAP_INDEX, 0x00, (uint8_t)4);
}

int Epos4Controller::_set_operational_state() {
    std::cout << "[EposDriver] Switching to OPERATIONAL..." << std::endl;
    ethercat_context.slavelist[0].state = EC_STATE_OPERATIONAL;
    ecx_writestate(&ethercat_context, 0);

    int chk = 200;
    do {
        ecx_send_processdata(&ethercat_context);
        ecx_receive_processdata(&ethercat_context, EC_TIMEOUTRET);
        ecx_statecheck(&ethercat_context, 0, EC_STATE_OPERATIONAL, 50000);
    } while (chk-- && (ethercat_context.slavelist[0].state != EC_STATE_OPERATIONAL));

    if (ethercat_context.slavelist[0].state != EC_STATE_OPERATIONAL) {
        return -1;
    }
    return 0;
}

void Epos4Controller::_set_motor_shared_data(int slave_index) {
    auto data = std::make_unique<SharedMotorData>();
    data->pdo_output = (PdoOutput *)ethercat_context.slavelist[slave_index].outputs;
    data->pdo_input = (PdoInput *)ethercat_context.slavelist[slave_index].inputs;
    motor_data_list.push_back(std::move(data));
}

void Epos4Controller::_configure_slaves_in_network() {
    connected_slave_count = ethercat_context.slavecount;
    std::cout << "[EposDriver] Found " << connected_slave_count << " slaves." << std::endl;

    for (int slave_index = 1; slave_index <= connected_slave_count; slave_index++) {
        ethercat_context.slavelist[slave_index].state = EC_STATE_PRE_OP;
        ecx_writestate(&ethercat_context, slave_index);

        int retries = 0;
        while (retries++ < 100 && ethercat_context.slavelist[slave_index].state != EC_STATE_PRE_OP) {
            ecx_statecheck(&ethercat_context, slave_index, EC_STATE_PRE_OP, 20000);
        }
        _set_slave_default_values(slave_index);
    }
}

void Epos4Controller::_set_slave_default_values(int slave_index) {
    // Set Default Motion Profile
    _sdo_write(slave_index, PROFILE_VEL, 0x00, (uint32_t)1000);
    _sdo_write(slave_index, PROFILE_ACC, 0x00, (uint32_t)10000);
    _sdo_write(slave_index, PROFILE_DEC, 0x00, (uint32_t)10000);

    _configure_pdo(slave_index);
}

void Epos4Controller::_state_machine(SharedMotorData &motor, uint16_t status) {
    if (status & 0x0008)
        motor.pdo_output->control_word = 0x0080; // Fault -> Reset
    else if ((status & 0x004F) == 0x0040)
        motor.pdo_output->control_word = 0x0006; // Shutdown
    else if ((status & 0x006F) == 0x0021)
        motor.pdo_output->control_word = 0x0007; // Switch On
    else if ((status & 0x006F) == 0x0023)
        motor.pdo_output->control_word = 0x000F; // Enable
}

void Epos4Controller::_start_movement(SharedMotorData &motor, uint16_t status) {
    if ((status & 0x006F) == 0x0027) {
        motor.pdo_output->target_position = motor.new_target_position.load();
        if (motor.has_new_command.exchange(false)) {
            // 0x003F = Enable + New Setpoint + Change Immediately
            motor.pdo_output->control_word = 0x003F;
        } else if (status & 0x1000) {
            // Handshake Acknowledged: return to normal Enable
            motor.pdo_output->control_word = 0x000F;
        }
    }
}

void Epos4Controller::_wait_for_motors_enabled() {
    std::cout << "[EposDriver] Waiting for motors to reach Operation Enabled..." << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    bool all_ready = false;

    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(5)) {
        all_ready = true;

        for (int i = 0; i < connected_slave_count; i++) {
            uint16_t status = motor_data_list[i]->pdo_input->status_word;

            if ((status & 0x006F) != 0x0027) {
                all_ready = false;
                break;
            }
        }

        if (all_ready) {
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    is_driver_running = false;
    if (worker.joinable())
        worker.join();
    throw std::runtime_error("Timeout: Motors failed to enable within 5 seconds.");
}

void Epos4Controller::_worker_loop() {
    while (is_driver_running) {
        try {
            auto start_time = std::chrono::steady_clock::now();

            // PDO Exchange
            {
                std::lock_guard<std::mutex> lock(ecat_mutex);
                ecx_send_processdata(&ethercat_context);
                ecx_receive_processdata(&ethercat_context, EC_TIMEOUTRET);
            }

            if (ethercat_context.slavelist[0].state == EC_STATE_OPERATIONAL) {
                for (int i = 0; i < connected_slave_count; i++) {
                    auto &motor = *motor_data_list[i];

                    // Read PDO
                    motor.position = motor.pdo_input->actual_position;
                    uint16_t status = motor.pdo_input->status_word;

                    // Logic
                    _state_machine(motor, status);
                    motor.pdo_output->operation_mode = MODE_PROFILE_POSITION;
                    _start_movement(motor, status);
                }
            }
            std::this_thread::sleep_until(start_time + std::chrono::milliseconds(1));

        } catch (const std::exception &e) {
            std::cerr << "[Worker Error] " << e.what() << std::endl;
        }
    }
}

// --- Public API ---

void Epos4Controller::set_target_position(int motor_id, int32_t position_value) {
    if (motor_id < 1 || motor_id > connected_slave_count)
        return;
    auto &motor = *motor_data_list[motor_id - 1];
    motor.new_target_position = position_value;
    motor.has_new_command = true;
}

int32_t Epos4Controller::get_current_position(int motor_id) {
    if (motor_id < 1 || motor_id > connected_slave_count)
        return 0;
    return motor_data_list[motor_id - 1]->position.load();
}

void Epos4Controller::set_profile_velocity(int motor_id, uint32_t velocity) {
    if (motor_id < 1 || motor_id > connected_slave_count)
        return;
    _sdo_write(motor_id, PROFILE_VEL, 0x00, velocity);
}

int Epos4Controller::get_slave_count() const { return connected_slave_count; }
