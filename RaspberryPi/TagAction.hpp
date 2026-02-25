#ifndef TAG_ACTION_HPP
#define TAG_ACTION_HPP

#include <iostream>
#include <chrono>
#include <string>

class TagAction {
private:
    static inline int last_id = -1;
    static inline auto last_trigger_time = std::chrono::steady_clock::now();

public:
    static std::string get_description(int id) {
        if (id == 0) return "HOME BASE: Landing Zone";
        if (id == 10) return "SUPPLY DROP: Release Area";
        if (id == 20) return "WAYPOINT: Course Correction";
        return "UNKNOWN: Generic Target";
    }

    static void execute(int tag_id) {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_trigger_time);

        // Only trigger action every 3 seconds per tag to avoid spam
        if (tag_id != last_id || duration.count() > 3) {
            std::cout << "[ACTION] Triggered ID " << tag_id << ": " << get_description(tag_id) << std::endl;
            last_id = tag_id;
            last_trigger_time = now;
        }
    }
};

#endif
