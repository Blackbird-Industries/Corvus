#define CROW_MAIN
#include "crow_all.h"
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>

using namespace mavsdk;

// --- GLOBAL STATE ---
std::atomic<bool> is_running{true};
std::vector<uchar> global_frame_buffer;
std::mutex frame_mtx;
std::atomic<int> latest_tag_id{-1};
std::atomic<float> drone_alt{0.0f};
std::atomic<float> drone_bat{0.0f};
std::atomic<bool> fc_connected{false};

// --- CAMERA & APRILTAG WORKER ---
void camera_worker() {
    // This is your "Golden Pipeline" that worked!
    std::string pipeline = "libcamerasrc ! video/x-raw, width=640, height=480, format=NV12 ! videoconvert ! video/x-raw, format=BGR ! appsink drop=true max-buffers=1";
    cv::VideoCapture cap;

    // AprilTag Setup
    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    while (is_running) {
        if (!cap.isOpened()) {
            std::cout << "[CAMERA] Connecting to IMX415..." << std::endl;
            cap.open(pipeline, cv::CAP_GSTREAMER);
            if (!cap.isOpened()) {
                std::this_thread::sleep_for(std::chrono::seconds(2));
                continue;
            }
        }

        cv::Mat frame, gray;
        if (cap.grab() && cap.retrieve(frame) && !frame.empty()) {
            // AprilTag Detection Logic
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
            image_u8_t img = { .width = gray.cols, .height = gray.rows, .stride = gray.cols, .buf = gray.data };
            zarray_t *detections = apriltag_detector_detect(td, &img);

            int found_id = -1;
            for (int i = 0; i < zarray_size(detections); i++) {
                apriltag_detection_t *det;
                zarray_get(detections, i, &det);
                found_id = det->id;
                // Draw a simple box on the frame
                for (int j=0; j<4; j++) {
                    cv::line(frame, cv::Point(det->p[j][0], det->p[j][1]),
                             cv::Point(det->p[(j+1)%4][0], det->p[(j+1)%4][1]),
                             cv::Scalar(0, 255, 0), 2);
                }
            }
            latest_tag_id = found_id;
            zarray_destroy(detections);

            // Encode for Web
            std::vector<uchar> buf;
            cv::imencode(".jpg", frame, buf, {cv::IMWRITE_JPEG_QUALITY, 70});
            
            std::lock_guard<std::mutex> lock(frame_mtx);
            global_frame_buffer = std::move(buf);
        } else {
            cap.release();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
    
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
}

// --- TELEMETRY WORKER (MAVSDK v3) ---
void telemetry_worker() {
    Mavsdk mavsdk{Mavsdk::Configuration{ComponentType::GroundStation}};
    ConnectionResult conn_res = mavsdk.add_any_connection("serial:///dev/ttyACM0:115200");

    if (conn_res != ConnectionResult::Success) {
        std::cout << "[TELEMETRY] Pixhawk not found. Offline mode." << std::endl;
        return; 
    }

    while (mavsdk.systems().empty()) { std::this_thread::sleep_for(std::chrono::seconds(1)); }
    
    auto system = mavsdk.systems()[0];
    auto telemetry = Telemetry{system};
    fc_connected = true;

    telemetry.subscribe_position([](Telemetry::Position pos) { drone_alt = pos.relative_altitude_m; });
    telemetry.subscribe_battery([](Telemetry::Battery bat) { drone_bat = bat.remaining_percent * 100.0f; });

    while (is_running) { std::this_thread::sleep_for(std::chrono::seconds(1)); }
}

int main() {
    crow::SimpleApp app;

    // The logic to find index.html if you want to use a file, 
    // but I'll keep the route internal for now for maximum stability.
    CROW_ROUTE(app, "/")([](){
        return "<html><body style='background:#121212;color:white;text-align:center;font-family:sans-serif;'>"
               "<h1>CORVUS GCS</h1>"
               "<div style='font-size:1.5em;'>ALT: <span id='alt'>0</span>m | BAT: <span id='bat'>0</span>% | TAG: <span id='tag'>-1</span></div>"
               "<img id='stream' src='' style='margin-top:20px; border:2px solid #444;'>"
               "<script>"
               "const img = document.getElementById('stream');"
               "function loadFrame() { img.src = '/snapshot?t=' + new Date().getTime(); }"
               "img.onload = () => setTimeout(loadFrame, 30);"
               "loadFrame();"
               "setInterval(() => { fetch('/api/status').then(r=>r.json()).then(d=>{ "
               "document.getElementById('alt').innerText=d.alt.toFixed(1);"
               "document.getElementById('bat').innerText=d.bat;"
               "document.getElementById('tag').innerText=d.tag_id;"
               "}); }, 200);"
               "</script></body></html>";
    });

    CROW_ROUTE(app, "/snapshot")([](){
        std::lock_guard<std::mutex> lock(frame_mtx);
        if (global_frame_buffer.empty()) return crow::response(404);
        crow::response res(std::string(global_frame_buffer.begin(), global_frame_buffer.end()));
        res.set_header("Content-Type", "image/jpeg");
        return res;
    });

    CROW_ROUTE(app, "/api/status")([](){
        crow::json::wvalue x;
        x["alt"] = (float)drone_alt;
        x["bat"] = (int)drone_bat;
        x["tag_id"] = (int)latest_tag_id;
        x["connected"] = (bool)fc_connected;
        return x;
    });

    std::thread cam_thread(camera_worker);
    std::thread tel_thread(telemetry_worker);

    app.port(5000).bindaddr("0.0.0.0").multithreaded().run();

    is_running = false;
    if(cam_thread.joinable()) cam_thread.join();
    if(tel_thread.joinable()) tel_thread.join();
}
