#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <opencv2/opencv.hpp>
#include "crow_all.h"
#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <fstream>

using namespace mavsdk;

struct DroneState {
    float alt = 0.0f, batt = 0.0f, roll = 0.0f, pitch = 0.0f;
    std::mutex mtx;
};

DroneState state;
std::vector<uchar> global_frame_buffer;
std::mutex frame_mtx;

int main() {
    Mavsdk mavsdk{Mavsdk::Configuration{1, 190, false}};
    // Using serial0 (GPIO 14/15)
    mavsdk.add_any_connection("serial:///dev/serial0:57600");

    // --- TELEMETRY THREAD ---
    std::thread nav_thread([&mavsdk]() {
        std::cout << "[MAVSDK] Waiting for system..." << std::endl;
        while (mavsdk.systems().empty()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        auto system = mavsdk.systems().at(0);
        // We define telemetry here so it stays alive as long as the thread runs
        auto telemetry = Telemetry{system};

        std::cout << "[MAVSDK] System found! Requesting streams..." << std::endl;
        telemetry.set_rate_position(5.0);
        telemetry.set_rate_attitude_euler(10.0);
        telemetry.set_rate_battery(1.0);

        // Subscriptions
        telemetry.subscribe_position([](Telemetry::Position pos) {
            std::lock_guard<std::mutex> lock(state.mtx);
            state.alt = pos.relative_altitude_m;
        });

        telemetry.subscribe_attitude_euler([](Telemetry::EulerAngle angle) {
            std::lock_guard<std::mutex> lock(state.mtx);
            state.roll = angle.roll_deg;
            state.pitch = angle.pitch_deg;
        });

        telemetry.subscribe_battery([](Telemetry::Battery batt) {
            std::lock_guard<std::mutex> lock(state.mtx);
            state.batt = batt.remaining_percent * 100.0f;
        });

        // Keep the thread alive so the telemetry object doesn't die
        while (true) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });

    // --- CAMERA THREAD ---
    std::thread cam_thread([]() {
        cv::VideoCapture cap(0);
        if (!cap.isOpened()) return;
        cv::Mat frame;
        while (true) {
            cap >> frame;
            if (frame.empty()) continue;
            std::vector<uchar> buf;
            cv::imencode(".jpg", frame, buf, {cv::IMWRITE_JPEG_QUALITY, 60});
            {
                std::lock_guard<std::mutex> lock(frame_mtx);
                global_frame_buffer = std::move(buf);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(40));
        }
    });

    // --- WEB SERVER ---
    crow::SimpleApp app;

    CROW_ROUTE(app, "/")([](){
        std::ifstream f("/home/blackbird/Corvus/RaspberryPi/index.html");
        if(!f) return crow::response(404, "Missing index.html");
        std::stringstream ss; ss << f.rdbuf();
        return crow::response(ss.str());
    });

    CROW_ROUTE(app, "/api/sensors")([](){
        std::lock_guard<std::mutex> lock(state.mtx);
        crow::json::wvalue x;
        x["alt"] = state.alt; 
        x["batt"] = state.batt;
        x["roll"] = state.roll; 
        x["pitch"] = state.pitch;
        return x;
    });

    CROW_ROUTE(app, "/live_frame")([](const crow::request&, crow::response& res) {
        std::vector<uchar> buf;
        {
            std::lock_guard<std::mutex> lock(frame_mtx);
            buf = global_frame_buffer;
        }
        res.set_header("Content-Type", "image/jpeg");
        res.write(std::string(buf.begin(), buf.end()));
        res.end();
    });

    app.port(8080).concurrency(10).run();

    return 0;
}
