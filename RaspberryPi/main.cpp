#include "crow_all.h"
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <vector>
#include <mutex>
#include <atomic>

using namespace mavsdk;

// Global Shared Data
std::mutex frame_mutex;
cv::Mat global_frame;
std::atomic<int> latest_tag_id{-1};
std::atomic<float> drone_alt{0.0f};
std::atomic<float> drone_bat{0.0f};

// --- MAVLink Thread: Pulls data from Pixhawk ---
void telemetry_worker() {
    Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::GroundStation}};
    // Connect to Pixhawk via USB
    ConnectionResult conn_res = mavsdk.add_any_connection("serial:///dev/ttyACM0:115200");
    
    if (conn_res != ConnectionResult::Success) return;

    // Wait for the drone to be discovered
    while (mavsdk.systems().empty()) { std::this_thread::sleep_for(std::chrono::seconds(1)); }
    auto system = mavsdk.systems()[0];
    auto telemetry = Telemetry{system};

    // Subscribe to Altitude
    telemetry.subscribe_position([](Telemetry::Position pos) {
        drone_alt = pos.relative_altitude_m;
    });

    // Subscribe to Battery
    telemetry.subscribe_battery([](Telemetry::Battery bat) {
        drone_bat = bat.remaining_percent * 100.0f;
    });

    // Keep thread alive
    while (true) { std::this_thread::sleep_for(std::chrono::seconds(1)); }
}

// --- Camera Thread: AprilTags + OpenCV ---
void camera_worker() {
    cv::VideoCapture cap(0);
    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    cv::Mat frame, gray;
    while (true) {
        cap >> frame;
        if (frame.empty()) continue;

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        image_u8_t img = { .width = gray.cols, .height = gray.rows, .stride = gray.cols, .buf = gray.data };
        zarray_t *detections = apriltag_detector_detect(td, &img);

        latest_tag_id = (zarray_size(detections) > 0) ? 0 : -1;
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            latest_tag_id = det->id;
            // Draw Box
            for (int j=0; j<4; j++)
                cv::line(frame, cv::Point(det->p[j][0], det->p[j][1]), cv::Point(det->p[(j+1)%4][0], det->p[(j+1)%4][1]), cv::Scalar(0,255,0), 2);
        }

        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            frame.copyTo(global_frame);
        }
        zarray_destroy(detections);
    }
}

int main() {
    crow::SimpleApp app;

    std::thread cam_t(camera_worker);
    std::thread tel_t(telemetry_worker);

    CROW_ROUTE(app, "/api/data")([]{
        crow::json::wvalue x;
        x["tag_id"] = (int)latest_tag_id;
        x["alt"] = (float)drone_alt;
        x["bat"] = (int)drone_bat;
        return x;
    });

    CROW_ROUTE(app, "/video")([&](const crow::request&, crow::response& res){
        res.set_header("Content-Type", "multipart/x-mixed-replace; boundary=frame");
        while (true) {
            std::vector<uchar> buf;
            {
                std::lock_guard<std::mutex> lock(frame_mutex);
                if (global_frame.empty()) continue;
                cv::imencode(".jpg", global_frame, buf);
            }
            res.write("--frame\r\nContent-Type: image/jpeg\r\n\r\n" + std::string(buf.begin(), buf.end()) + "\r\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(40));
        }
    });

    app.port(8080).run();
}
