#include <fstream>
#include <thread>
#include <chrono>

int main() {
    std::ofstream out("count.txt", std::ios::out | std::ios::trunc);

    if (!out.is_open()) {
        return 1;
    }

    for (int i = 1; i <= 10; ++i) {
        out << "Count: " << i << std::endl;
        out.flush();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    out.close();
    return 0;
}