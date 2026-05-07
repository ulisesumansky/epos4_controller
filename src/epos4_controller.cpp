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
        ecx_close(&ethercat_context);
        throw std::runtime_error("No slaves found on bus.");
    }

    _configure_slaves_in_network();
    ecx_config_map_group(&ethercat_context, io_map_buffer, 0);
    ecx_configdc(&ethercat_context);

    for (int slave_index = 1; slave_index <= connected_slave_count; slave_index++) {
        _set_motor_shared_data(slave_index);
    }

    if (!_set_operational_state()) {
        ecx_close(&ethercat_context);
        throw std::runtime_error("Could not switch to Operational State.");
    }

    is_driver_running = true;
    worker = std::thread(&Epos4Controller::_worker_loop, this);
    _wait_for_motors_enabled();
    std::cout << "[EposDriver] System Running." << std::endl;
}

Epos4Controller::~Epos4Controller() {
    is_driver_running = false;
    if (worker.joinable()) {
        worker.join();
    }
    ecx_close(&ethercat_context);
    std::cout << "[EposDriver] Closed." << std::endl;
}

void Epos4Controller::_validate_motor_id(int motor_id) const {
    if (motor_id < 1 || motor_id > connected_slave_count) {
        throw std::out_of_range("motor_id " + std::to_string(motor_id) + " out of range [1, " +
                                std::to_string(connected_slave_count) + "]");
    }
}

void Epos4Controller::_configure_pdo(int slave_index) {
    _sdo_write(slave_index, RX_PDO_MAP_INDEX, 0x00, (uint8_t)0);
    _sdo_write(slave_index, RX_PDO_MAP_INDEX, 0x01, MAP_OBJ_CONTROL_WORD);
    _sdo_write(slave_index, RX_PDO_MAP_INDEX, 0x02, MAP_OBJ_OP_MODE);
    _sdo_write(slave_index, RX_PDO_MAP_INDEX, 0x03, MAP_OBJ_TARGET_POS);
    _sdo_write(slave_index, RX_PDO_MAP_INDEX, 0x00, (uint8_t)3);

    _sdo_write(slave_index, TX_PDO_MAP_INDEX, 0x00, (uint8_t)0);
    _sdo_write(slave_index, TX_PDO_MAP_INDEX, 0x01, MAP_OBJ_STATUS_WORD);
    _sdo_write(slave_index, TX_PDO_MAP_INDEX, 0x02, MAP_OBJ_MODE_DISP);
    _sdo_write(slave_index, TX_PDO_MAP_INDEX, 0x03, MAP_OBJ_ACTUAL_POS);
    _sdo_write(slave_index, TX_PDO_MAP_INDEX, 0x04, MAP_OBJ_ERROR_CODE);
    _sdo_write(slave_index, TX_PDO_MAP_INDEX, 0x00, (uint8_t)4);
}

bool Epos4Controller::_set_operational_state() {
    std::cout << "[EposDriver] Switching to OPERATIONAL..." << std::endl;
    ethercat_context.slavelist[0].state = EC_STATE_OPERATIONAL;
    ecx_writestate(&ethercat_context, 0);

    int chk = 200;
    do {
        ecx_send_processdata(&ethercat_context);
        ecx_receive_processdata(&ethercat_context, EC_TIMEOUTRET);
        ecx_statecheck(&ethercat_context, 0, EC_STATE_OPERATIONAL, 50000);
    } while (chk-- && (ethercat_context.slavelist[0].state != EC_STATE_OPERATIONAL));

    return ethercat_context.slavelist[0].state == EC_STATE_OPERATIONAL;
}

void Epos4Controller::_set_motor_shared_data(int slave_index) {
    auto data = std::make_unique<SharedMotorData>();
    data->pdo_output = (PdoOutput *)ethercat_context.slavelist[slave_index].outputs;
    data->pdo_input = (PdoInput *)ethercat_context.slavelist[slave_index].inputs;

    if (!data->pdo_output || !data->pdo_input) {
        throw std::runtime_error("PDO mapping failed for slave " + std::to_string(slave_index) +
                                 ": null pointer from SOEM");
    }

    motor_data_list.push_back(std::move(data));
}

void Epos4Controller::_configure_slaves_in_network() {
    connected_slave_count = ethercat_context.slavecount;
    std::cout << "[EposDriver] Found " << connected_slave_count << " slaves." << std::endl;

    for (int slave_index = 1; slave_index <= connected_slave_count; slave_index++) {
        ethercat_context.slavelist[slave_index].state = EC_STATE_PRE_OP;
        ecx_writestate(&ethercat_context, slave_index);

        int retries = 0;
        while (retries++ < 100 &&
               ethercat_context.slavelist[slave_index].state != EC_STATE_PRE_OP) {
            ecx_statecheck(&ethercat_context, slave_index, EC_STATE_PRE_OP, 20000);
        }
        _set_slave_default_values(slave_index);
    }
}

void Epos4Controller::_set_slave_default_values(int slave_index) {
    _sdo_write(slave_index, PROFILE_VEL, 0x00, DEFAULT_PROFILE_VELOCITY);
    _sdo_write(slave_index, PROFILE_ACC, 0x00, DEFAULT_PROFILE_ACCELERATION);
    _sdo_write(slave_index, PROFILE_DEC, 0x00, DEFAULT_PROFILE_DECELERATION);

    _configure_pdo(slave_index);
}

MotorState Epos4Controller::_decode_state(uint16_t status) {
    if (status & STATUS_FAULT_BIT)
        return MotorState::Fault;
    if ((status & STATUS_MASK_OPERATION_ENABLED) == STATUS_VAL_OPERATION_ENABLED)
        return MotorState::OperationEnabled;
    if ((status & STATUS_MASK_SWITCHED_ON) == STATUS_VAL_SWITCHED_ON)
        return MotorState::SwitchedOn;
    if ((status & STATUS_MASK_READY_TO_SWITCH_ON) == STATUS_VAL_READY_TO_SWITCH_ON)
        return MotorState::ReadyToSwitchOn;
    if ((status & STATUS_MASK_SWITCH_ON_DISABLED) == STATUS_VAL_SWITCH_ON_DISABLED)
        return MotorState::SwitchOnDisabled;
    return MotorState::Unknown;
}

void Epos4Controller::_enable_motor(SharedMotorData &motor, uint16_t status) {
    MotorState state = _decode_state(status);

    if (state == MotorState::Fault && !motor.in_fault.exchange(true)) {
        motor.fault_count.fetch_add(1);
        if (motor.consecutive_faults.fetch_add(1) + 1 >= MAX_CONSECUTIVE_FAULTS) {
            motor.fault_locked = true;
            std::cerr << "[EposDriver] Motor locked after " << MAX_CONSECUTIVE_FAULTS
                      << " consecutive faults" << std::endl;
        }
    } else if (state != MotorState::Fault) {
        motor.in_fault = false;
        motor.consecutive_faults = 0;
    }

    if (motor.fault_locked)
        return;

    switch (state) {
    case MotorState::Fault:
        motor.pdo_output->control_word = CMD_FAULT_RESET;
        break;
    case MotorState::SwitchOnDisabled:
        motor.pdo_output->control_word = CMD_SHUTDOWN;
        break;
    case MotorState::ReadyToSwitchOn:
        motor.pdo_output->control_word = CMD_SWITCH_ON;
        break;
    case MotorState::SwitchedOn:
        motor.pdo_output->control_word = CMD_ENABLE_OPERATION;
        break;
    default:
        break;
    }
}

void Epos4Controller::_start_movement(SharedMotorData &motor, uint16_t status) {
    if ((status & STATUS_MASK_OPERATION_ENABLED) != STATUS_VAL_OPERATION_ENABLED)
        return;

    motor.pdo_output->target_position = motor.new_target_position.load();
    if (motor.has_new_command.exchange(false)) {
        uint16_t cmd = CMD_START_MOVEMENT;
        if (motor.move_relative.load())
            cmd |= CTRL_BIT_ABS_REL;
        else
            cmd &= ~CTRL_BIT_ABS_REL;
        motor.pdo_output->control_word = cmd;
    } else if (status & STATUS_SETPOINT_ACK) {
        motor.pdo_output->control_word = CMD_ENABLE_OPERATION;
    }
}

void Epos4Controller::_wait_for_motors_enabled() {
    std::cout << "[EposDriver] Waiting for motors to reach Operation Enabled..." << std::endl;

    auto start_time = std::chrono::steady_clock::now();

    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(5)) {
        bool all_ready = true;

        for (int i = 0; i < connected_slave_count; i++) {
            if (_decode_state(motor_data_list[i]->pdo_input->status_word) !=
                MotorState::OperationEnabled) {
                all_ready = false;
                break;
            }
        }

        if (all_ready)
            return;

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

            {
                std::lock_guard<std::mutex> lock(ecat_mutex);
                ecx_send_processdata(&ethercat_context);
                ecx_receive_processdata(&ethercat_context, EC_TIMEOUTRET);
            }

            if (ethercat_context.slavelist[0].state == EC_STATE_OPERATIONAL) {
                for (int i = 0; i < connected_slave_count; i++) {
                    auto &motor = *motor_data_list[i];

                    motor.position = motor.pdo_input->actual_position;
                    uint16_t status = motor.pdo_input->status_word;

                    _enable_motor(motor, status);
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

void Epos4Controller::set_target_position(int motor_id, int32_t position_value, bool relative) {
    _validate_motor_id(motor_id);
    auto &motor = *motor_data_list[motor_id - 1];
    motor.new_target_position = position_value;
    motor.move_relative = relative;
    motor.has_new_command = true;
}

int32_t Epos4Controller::get_current_position(int motor_id) {
    _validate_motor_id(motor_id);
    return motor_data_list[motor_id - 1]->position.load();
}

void Epos4Controller::set_profile_velocity(int motor_id, uint32_t velocity) {
    _validate_motor_id(motor_id);
    std::lock_guard<std::mutex> lock(ecat_mutex);
    _sdo_write(motor_id, PROFILE_VEL, 0x00, velocity);
}

uint32_t Epos4Controller::get_profile_velocity(int motor_id) {
    _validate_motor_id(motor_id);
    std::lock_guard<std::mutex> lock(ecat_mutex);
    return _sdo_read<uint32_t>(motor_id, PROFILE_VEL, 0x00);
}

MotorState Epos4Controller::get_motor_state(int motor_id) {
    _validate_motor_id(motor_id);
    auto &motor = *motor_data_list[motor_id - 1];

    if (motor.fault_locked.load())
        return MotorState::FaultLocked;

    return _decode_state(motor.pdo_input->status_word);
}

uint32_t Epos4Controller::get_fault_count(int motor_id) {
    _validate_motor_id(motor_id);
    return motor_data_list[motor_id - 1]->fault_count.load();
}

bool Epos4Controller::is_motor_faulted(int motor_id) {
    _validate_motor_id(motor_id);
    return motor_data_list[motor_id - 1]->fault_locked.load();
}

void Epos4Controller::clear_fault(int motor_id) {
    _validate_motor_id(motor_id);
    auto &motor = *motor_data_list[motor_id - 1];
    motor.consecutive_faults = 0;
    motor.fault_locked = false;
}

int Epos4Controller::get_slave_count() const { return connected_slave_count; }
