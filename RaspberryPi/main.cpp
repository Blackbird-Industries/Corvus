#include "crow_all.h"
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <vector>
#include <mutex>
#include <atomic>
#include <thread>

using namespace mavsdk;

// --- Global State ---
std::mutex frame_mutex;
cv::Mat global_frame;
std::atomic<int> latest_tag_id{-1};
std::atomic<float> drone_alt{0.0f};
std::atomic<float> drone_bat{0.0f};
std::atomic<bool> is_connected{false};

// --- Telemetry Thread: Talk to Pixhawk ---
void telemetry_worker() {
    Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::GroundStation}};
    // Connect via USB-C (usually /dev/ttyACM0)
    ConnectionResult conn_res = mavsdk.add_any_connection("serial:///dev/ttyACM0:115200");
    
    if (conn_res != ConnectionResult::Success) return;

    // Wait for the drone to appear
    while (mavsdk.systems().empty()) { std::this_thread::sleep_for(std::chrono::seconds(1)); }
    
    auto system = mavsdk.systems()[0];
    auto telemetry = Telemetry{system};
    is_connected = true;

    telemetry.subscribe_position([](Telemetry::Position pos) {
        drone_alt = pos.relative_altitude_m;
    });

    telemetry.subscribe_battery([](Telemetry::Battery bat) {
        drone_bat = bat.remaining_percent * 100.0f;
    });

    while (true) { std::this_thread::sleep_for(std::chrono::seconds(1)); }
}

// --- Camera Thread: AprilTags + OpenCV ---
void camera_worker() {
    cv::VideoCapture cap(0); // 0 for default Pi camera
    if (!cap.isOpened()) return;

    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    cv::Mat frame, gray;
    while (true) {
        cap >> frame;
        if (frame.empty()) continue;

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Map OpenCV Mat to AprilTag structure
        image_u8_t img = { .width = gray.cols, .height = gray.rows, 
                           .stride = gray.cols, .buf = gray.data };

        zarray_t *detections = apriltag_detector_detect(td, &img);
        
        int found_id = -1;
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            found_id = det->id;

            // Draw bounding box on the frame
            for (int j=0; j<4; j++) {
                cv::line(frame, cv::Point(det->p[j][0], det->p[j][1]),
                         cv::Point(det->p[(j+1)%4][0], det->p[(j+1)%4][1]),
                         cv::Scalar(0, 255, 0), 2);
            }
        }
        latest_tag_id = found_id;

        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            frame.copyTo(global_frame);
        }
        zarray_destroy(detections);
    }
}

// --- Main App: Crow Server ---
int main() {
    crow::SimpleApp app;

    std::thread cam_t(camera_worker);
    std::thread tel_t(telemetry_worker);

    // API: Unified Data Endpoint
    CROW_ROUTE(app, "/api/status")([]{
        crow::json::wvalue x;
        x["tag_id"] = (int)latest_tag_id;
        x["alt"] = (float)drone_alt;
        x["bat"] = (int)drone_bat;
        x["fc_connected"] = (bool)is_connected;
        return x;
    });

    // API: Video Stream
    CROW_ROUTE(app, "/video")([&](const crow::request&, crow::response& res){
        res.set_header("Content-Type", "multipart/x-mixed-replace; boundary=frame");
        while (true) {
            std::vector<uchar> buf;
            {
                std::lock_guard<std::mutex> lock(frame_mutex);
                if (global_frame.empty()) continue;
                cv::imencode(".jpg", global_frame, buf);
            }
            std::string out = "--frame\r\nContent-Type: image/jpeg\r\n\r\n" + 
                              std::string(buf.begin(), buf.end()) + "\r\n";
            res.write(out);
            std::this_thread::sleep_for(std::chrono::milliseconds(40));
        }
    });

    app.port(8080).run();
}
