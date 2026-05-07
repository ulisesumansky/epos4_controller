#pragma once

#include <cstddef>
#include <cstdint>

namespace EposConsts {

// --- CiA 402 Object Dictionary Addresses ---
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

// --- CiA 402 Status Word Masks ---
constexpr uint16_t STATUS_FAULT_BIT = 0x0008;
constexpr uint16_t STATUS_MASK_SWITCH_ON_DISABLED = 0x004F;
constexpr uint16_t STATUS_VAL_SWITCH_ON_DISABLED = 0x0040;
constexpr uint16_t STATUS_MASK_READY_TO_SWITCH_ON = 0x006F;
constexpr uint16_t STATUS_VAL_READY_TO_SWITCH_ON = 0x0021;
constexpr uint16_t STATUS_MASK_SWITCHED_ON = 0x006F;
constexpr uint16_t STATUS_VAL_SWITCHED_ON = 0x0023;
constexpr uint16_t STATUS_MASK_OPERATION_ENABLED = 0x006F;
constexpr uint16_t STATUS_VAL_OPERATION_ENABLED = 0x0027;
constexpr uint16_t STATUS_SETPOINT_ACK = 0x1000;

// --- CiA 402 Control Word Commands ---
constexpr uint16_t CMD_FAULT_RESET = 0x0080;
constexpr uint16_t CMD_SHUTDOWN = 0x0006;
constexpr uint16_t CMD_SWITCH_ON = 0x0007;
constexpr uint16_t CMD_ENABLE_OPERATION = 0x000F;
constexpr uint16_t CMD_START_MOVEMENT = 0x003F;

// --- Fault Management ---
constexpr uint32_t MAX_CONSECUTIVE_FAULTS = 5;

// --- Default Motion Profile ---
constexpr uint32_t DEFAULT_PROFILE_VELOCITY = 1000;
constexpr uint32_t DEFAULT_PROFILE_ACCELERATION = 10000;
constexpr uint32_t DEFAULT_PROFILE_DECELERATION = 10000;

// --- PDO Mapping Constants ---
// Format: Index (16bit) + SubIndex (8bit) + BitLength (8bit)
constexpr uint16_t RX_PDO_MAP_INDEX = 0x1600;
constexpr uint16_t TX_PDO_MAP_INDEX = 0x1A00;

constexpr uint32_t MAP_OBJ_CONTROL_WORD = 0x60400010;
constexpr uint32_t MAP_OBJ_OP_MODE = 0x60600008;
constexpr uint32_t MAP_OBJ_TARGET_POS = 0x607A0020;

constexpr uint32_t MAP_OBJ_STATUS_WORD = 0x60410010;
constexpr uint32_t MAP_OBJ_MODE_DISP = 0x60610008;
constexpr uint32_t MAP_OBJ_ACTUAL_POS = 0x60640020;
constexpr uint32_t MAP_OBJ_ERROR_CODE = 0x603F0010;

// --- IO Map ---
constexpr size_t IO_MAP_BUFFER_SIZE = 40960;

// --- CiA 402 Control Word Bits ---
constexpr uint16_t CTRL_BIT_ABS_REL = 0x0040;

} // namespace EposConsts

enum class MotorState {
    NotReadyToSwitchOn,
    SwitchOnDisabled,
    ReadyToSwitchOn,
    SwitchedOn,
    OperationEnabled,
    Fault,
    FaultLocked,
    Unknown,
};

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
