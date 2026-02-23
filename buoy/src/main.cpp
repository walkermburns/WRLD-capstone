#include "BNO055.h"

#include <iostream>
#include <string>
#include <random>
#include <chrono>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

int main() {

    IMU imu;

    try {
        while (true) {
            imu.read_sensor();
            usleep(200000);
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}