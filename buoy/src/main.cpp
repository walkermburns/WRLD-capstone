#include <iostream>
#include <string>
#include <random>
#include <chrono>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

// Include the generated header
#include "buoy.pb.h"

using namespace std;

int main() {
    // 1. Socket Configuration
    const char* SERVER_IP = "100.95.214.8";
    const int PORT = 8080;

    int sockfd;
    struct sockaddr_in servaddr;

    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Socket creation failed");
        return -1;
    }

    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(PORT);
    servaddr.sin_addr.s_addr = inet_addr(SERVER_IP);

    // 2. Random Number Setup
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<uint32_t> dist(0, 1000);

    cout << "Sending IMU data to " << SERVER_IP << ":" << PORT << "..." << endl;

    try {
        while (true) {
            // 3. Create and Populate Protobuf Message
            buoy_proto::IMU imu_msg;
            imu_msg.set_data(dist(gen));
            
            auto now = chrono::system_clock::now().time_since_epoch();
            imu_msg.set_timestamp(chrono::duration_cast<chrono::milliseconds>(now).count());

            // 4. Serialize to String
            string serialized_data;
            if (!imu_msg.SerializeToString(&serialized_data)) {
                cerr << "Failed to serialize data." << endl;
                continue;
            }

            // 5. Send over UDP
            sendto(sockfd, serialized_data.c_str(), serialized_data.length(),
                   0, (const struct sockaddr *)&servaddr, sizeof(servaddr));

            cout << "Sent: Data=" << imu_msg.data() << " TS=" << imu_msg.timestamp() << endl;

            usleep(500000); // Send every 500ms
        }
    } catch (const std::exception& e) {
        cerr << "Error: " << e.what() << endl;
    }

    close(sockfd);
    return 0;
}