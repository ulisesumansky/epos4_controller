# EPOS4 Controller

C++17 library for controlling Maxon EPOS4 motor drives over EtherCAT using [SOEM](https://github.com/OpenEtherCATsociety/SOEM).

The library runs an internal worker thread at ~1kHz that handles PDO exchange, CiA 402 state machine transitions, fault recovery, and motion commands. The user interacts with a thread-safe API.

## Requirements

- Linux with root permissions (raw socket access for EtherCAT)
- C++17 compiler (GCC >= 14)
- CMake >= 3.10
- SOEM >= 2.0.0

### Setup with Conda

```bash
conda env create -f environment.yml
conda activate epos4_controller
```

## Build

```bash
mkdir build && cd build
cmake .. -DCMAKE_PREFIX_PATH=$CONDA_PREFIX
make
```

## Usage

```cpp
#include "epos4_controller.hpp"

// Initialize on network interface (requires root)
Epos4Controller driver("eth0");

int motor = 1;

// Motion profile
driver.set_profile_velocity(motor, 1500);

// Absolute movement
driver.set_target_position(motor, 50000);

// Relative movement
driver.set_target_position(motor, -25000, true);

// Read state
int32_t pos = driver.get_current_position(motor);
MotorState state = driver.get_motor_state(motor);
uint32_t vel = driver.get_profile_velocity(motor);

// Fault management
if (driver.is_motor_faulted(motor)) {
    uint32_t faults = driver.get_fault_count(motor);
    driver.clear_fault(motor);
}

// Direct SDO access for any object dictionary entry
driver.sdo_write<uint32_t>(motor, 0x2210, 0x01, 4096);  // encoder resolution
auto value = driver.sdo_read<uint32_t>(motor, 0x2210, 0x01);
```

## API

| Method | Description |
|--------|-------------|
| `set_target_position(motor_id, position, relative)` | Set target position. `relative` defaults to `false` (absolute) |
| `get_current_position(motor_id)` | Read current position in encoder counts |
| `set_profile_velocity(motor_id, velocity)` | Set profile velocity via SDO |
| `get_profile_velocity(motor_id)` | Read current profile velocity via SDO |
| `get_motor_state(motor_id)` | Returns `MotorState` enum |
| `get_fault_count(motor_id)` | Total accumulated fault count |
| `is_motor_faulted(motor_id)` | `true` if motor is locked from consecutive faults |
| `clear_fault(motor_id)` | Unlock a fault-locked motor |
| `get_slave_count()` | Number of motors on the bus |
| `sdo_write<T>(slave, index, sub, value)` | Write any object dictionary entry |
| `sdo_read<T>(slave, index, sub)` | Read any object dictionary entry |

## Motor states

```
NotReadyToSwitchOn -> SwitchOnDisabled -> ReadyToSwitchOn -> SwitchedOn -> OperationEnabled
                                                                                |
                           Fault <-----------------------------------------------+
                             |
                     (auto-reset up to 5 times, then FaultLocked)
```

The worker thread automatically transitions motors to `OperationEnabled`. If a motor enters `Fault`, it auto-resets. After 5 consecutive faults without recovery, the motor is locked (`FaultLocked`) and stops retrying. Call `clear_fault()` to unlock after resolving the cause.

## Notes

- `motor_id` is 1-indexed (matches EtherCAT slave numbering)
- The constructor blocks until all motors reach `OperationEnabled` (5s timeout)
- Motors must be pre-configured (sensor type, encoder, regulation) via EPOS Studio or `sdo_write`/`sdo_read`
- All public methods throw on invalid `motor_id` (`std::out_of_range`) or SDO failure (`std::runtime_error`)
