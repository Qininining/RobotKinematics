#ifndef DEBUG_UTILS_H
#define DEBUG_UTILS_H

#include <iostream>
#include <chrono>
#include <string>
#include <iomanip> // For std::fixed and std::setprecision

/**
 * @brief A simple RAII (Resource Acquisition Is Initialization) timer.
 *        It starts timing upon construction and prints the elapsed time
 *        upon destruction.
 */
class SimpleTimer {
public:
    /**
     * @brief Constructs the timer and starts timing.
     * @param name A name for this timer instance, used in the output message.
     */
    SimpleTimer(const std::string& name = "Timer")
        : name_(name), start_time_(std::chrono::high_resolution_clock::now()) {
        std::cout << "[Timer: " << name_ << "] Started." << std::endl;
    }

    /**
     * @brief Destroys the timer and prints the elapsed time.
     */
    ~SimpleTimer() {
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time_);
        double milliseconds = duration.count() / 1000.0;
        std::cout << "[Timer: " << name_ << "] Ended. Elapsed time: "
                  << std::fixed << std::setprecision(3) << milliseconds << " ms ("
                  << duration.count() << " us)." << std::endl;
    }

private:
    std::string name_;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
};

/**
 * @brief Prints the current timestamp.
 * @param message An optional message to print before the timestamp.
 */
inline void printCurrentTime(const std::string& message = "Current time") {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::cout << message << ": " << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S") << std::endl;
}

// Example usage:
/*
#include "debug_utils.h"

void some_function() {
    SimpleTimer timer("some_function execution");
    // ... code to measure ...
}

int main() {
    printCurrentTime("Program started");
    {
        SimpleTimer main_block_timer("Main Block");
        some_function();
        // ... other operations ...
    }
    printCurrentTime("Program finished");
    return 0;
}
*/

#endif // DEBUG_UTILS_H