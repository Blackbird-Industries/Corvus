#define CROW_MAIN
#include "crow_all.h"
#include <opencv2/opencv.hpp>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <csignal>

using namespace mavsdk;

// --- GLOBAL STATE ---
std::atomic<bool> is_running{true};
std::vector<uchar> global_frame_buffer;
std::mutex frame_mtx;

struct DroneState {
    float alt = 0.0f;
    float batt = 0.0f;
    float roll = 0.0f;
    float pitch = 0.0f;
    std::mutex mtx;
} drone_state;

void signal_handler(int s) {
    is_running = false;
    std::cout << "\n[SYSTEM] Shutting down Corvus GCS..." << std::endl;
    exit(0);
}

// --- CAMERA WORKER (IMX415 Pi 5) ---
void camera_worker() {
    std::string pipeline = "libcamerasrc ! video/x-raw, width=640, height=480, format=NV12 ! videoconvert ! video/x-raw, format=BGR ! appsink drop=true max-buffers=1";
    cv::VideoCapture cap;

    while (is_running) {
        if (!cap.isOpened()) {
            cap.open(pipeline, cv::CAP_GSTREAMER);
            if (!cap.isOpened()) {
                std::this_thread::sleep_for(std::chrono::seconds(2));
                continue;
            }
        }

        cv::Mat frame;
        if (cap.grab()) { 
            if (cap.retrieve(frame) && !frame.empty()) {
                std::vector<uchar> buf;
                cv::imencode(".jpg", frame, buf, {cv::IMWRITE_JPEG_QUALITY, 60});
                std::lock_guard<std::mutex> lock(frame_mtx);
                global_frame_buffer = std::move(buf);
            }
        } else {
            cap.release(); 
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
}

// --- TELEMETRY WORKER (Pixhawk) ---
void telemetry_worker() {
    Mavsdk mavsdk{Mavsdk::Configuration{1, 190, false}};
    
    // Try 57600. If hexdump worked but this doesn't, change to 115200.
    ConnectionResult conn_res = mavsdk.add_any_connection("serial:///dev/serial0:57600");
    if (conn_res != ConnectionResult::Success) {
        std::cerr << "[MAVSDK] Connection failed: " << conn_res << std::endl;
        return;
    }

    std::cout << "[MAVSDK] Searching for Pixhawk..." << std::endl;
    while (is_running && mavsdk.systems().empty()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    if (!is_running) return;
    auto system = mavsdk.systems().at(0);
    auto telemetry = Telemetry{system};

    // Explicitly set rates (Requesting data from Pixhawk)
    telemetry.set_rate_position(2.0);
    telemetry.set_rate_attitude_euler(5.0); // Fixed function name
    telemetry.set_rate_battery(1.0);

    telemetry.subscribe_position([](Telemetry::Position pos) {
        std::lock_guard<std::mutex> lock(drone_state.mtx);
        drone_state.alt = pos.relative_altitude_m;
    });

    telemetry.subscribe_battery([](Telemetry::Battery batt) {
        std::lock_guard<std::mutex> lock(drone_state.mtx);
        drone_state.batt = batt.remaining_percent * 100.0f;
    });

    telemetry.subscribe_attitude_euler([](Telemetry::EulerAngle angle) {
        std::lock_guard<std::mutex> lock(drone_state.mtx);
        drone_state.roll = angle.roll_deg;
        drone_state.pitch = angle.pitch_deg;
        // Debug print to terminal to verify Pixhawk is actually talking
        // std::cout << "Pitch: " << angle.pitch_deg << " Roll: " << angle.roll_deg << std::endl;
    });

    while (is_running) { std::this_thread::sleep_for(std::chrono::seconds(1)); }
}

int main() {
    std::signal(SIGINT, signal_handler);
    crow::SimpleApp app;

    // --- 1. Dashboard (Indigo Link) ---
    CROW_ROUTE(app, "/")([](){
        std::ifstream f("/home/blackbird/Corvus/RaspberryPi/index.html");
        if(!f) return crow::response(404, "index.html not found! Check the path in main.cpp.");
        std::stringstream ss;
        ss << f.rdbuf();
        return crow::response(ss.str());
    });

    // --- 2. Snapshot API ---
    CROW_ROUTE(app, "/snapshot")([](){
        std::string data;
        {
            std::lock_guard<std::mutex> lock(frame_mtx);
            if (global_frame_buffer.empty()) return crow::response(404);
            data = std::string(global_frame_buffer.begin(), global_frame_buffer.end());
        }
        crow::response res(data);
        res.set_header("Content-Type", "image/jpeg");
        return res;
    });

    // --- 3. WebSocket (The Data Pipe) ---
    CROW_ROUTE(app, "/ws").websocket(&app)
        .onmessage([&](crow::websocket::connection& conn, const std::string& data, bool is_binary) {
            crow::json::wvalue msg;
            {
                std::lock_guard<std::mutex> lock(drone_state.mtx);
                msg["alt"] = drone_state.alt;
                msg["batt"] = drone_state.batt;
                msg["roll"] = drone_state.roll;   // Added
                msg["pitch"] = drone_state.pitch; // Added
            }
            conn.send_text(msg.dump());
        });

    std::thread c(camera_worker);
    std::thread t(telemetry_worker);
    c.detach();
    t.detach();

    std::cout << "[SERVER] Corvus GCS active on http://0.0.0.0:5000" << std::endl;
    app.port(5000).bindaddr("0.0.0.0").multithreaded().run();
}
