#include "epos4_controller.hpp"
#include <cmath>
#include <iostream>
#include <string>
#include <thread>

void wait_until_reached(Epos4Controller &driver, int motor_id, int32_t target_pos) {
    while (std::abs(target_pos - driver.get_current_position(motor_id)) > 100) {
        std::cout << "Motor [" << motor_id
                  << "] position: " << driver.get_current_position(motor_id) << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <network_interface>" << std::endl;
        return 1;
    }

    std::string interface_name = argv[1];

    try {
        Epos4Controller driver(interface_name);
        int motor_id = 1;

        std::cout << "Motor state: " << static_cast<int>(driver.get_motor_state(motor_id))
                  << std::endl;

        driver.set_profile_velocity(motor_id, 1500);
        std::cout << "Profile velocity: " << driver.get_profile_velocity(motor_id) << std::endl;

        // Absolute movement
        driver.set_target_position(motor_id, 50000);
        wait_until_reached(driver, motor_id, 50000);
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Relative movement
        driver.set_target_position(motor_id, -25000, true);
        wait_until_reached(driver, motor_id, 25000);
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Return to origin
        driver.set_target_position(motor_id, 0);
        wait_until_reached(driver, motor_id, 0);

        // Check fault status
        if (driver.is_motor_faulted(motor_id)) {
            std::cout << "Motor faulted after " << driver.get_fault_count(motor_id) << " faults"
                      << std::endl;
            driver.clear_fault(motor_id);
        }

    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
}
