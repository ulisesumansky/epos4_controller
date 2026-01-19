#pragma once

#include <cstdint>

namespace EposConsts {

// --- CiA 402 Object Dictionary Addresses (Logic) ---
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

// --- PDO Mapping Constants ---
// Format: Index (16bit) + SubIndex (8bit) + BitLength (8bit)
constexpr uint16_t RX_PDO_MAP_INDEX = 0x1600; // Receive PDO (Master -> Slave)
constexpr uint16_t TX_PDO_MAP_INDEX = 0x1A00; // Transmit PDO (Slave -> Master)

// Mapping Objects for RxPDO
constexpr uint32_t MAP_OBJ_CONTROL_WORD = 0x60400010; // 16-bit
constexpr uint32_t MAP_OBJ_OP_MODE = 0x60600008;      // 8-bit
constexpr uint32_t MAP_OBJ_TARGET_POS = 0x607A0020;   // 32-bit

// Mapping Objects for TxPDO
constexpr uint32_t MAP_OBJ_STATUS_WORD = 0x60410010; // 16-bit
constexpr uint32_t MAP_OBJ_MODE_DISP = 0x60610008;   // 8-bit
constexpr uint32_t MAP_OBJ_ACTUAL_POS = 0x60640020;  // 32-bit
constexpr uint32_t MAP_OBJ_ERROR_CODE = 0x603F0010;  // 16-bit

} // namespace EposConsts

// PDO Structures
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
