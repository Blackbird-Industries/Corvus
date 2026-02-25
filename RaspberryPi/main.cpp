#define CROW_MAIN
#include "crow_all.h"
#include <opencv2/opencv.hpp>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include "TagAction.hpp"
#include <thread>
#include <mutex>
#include <atomic>

using namespace mavsdk;

std::atomic<bool> is_running{true};
std::vector<uchar> global_frame_buffer;
std::mutex frame_mtx;

struct DroneState {
    float alt = 0.0f, batt = 0.0f, roll = 0.0f, pitch = 0.0f;
    int last_tag_id = -1;
    std::string tag_desc = "Scanning...";
    std::mutex mtx;
} drone_state;

void camera_worker() {
    // AprilTag Setup
    apriltag_family_t *tf = tag36h11_create();
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    std::string pipeline = "libcamerasrc ! video/x-raw, width=640, height=480, format=NV12 ! videoconvert ! video/x-raw, format=BGR ! appsink drop=true max-buffers=1";
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

    while (is_running) {
        cv::Mat frame, gray;
        if (cap.read(frame)) {
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
            image_u8_t img = { .width = gray.cols, .height = gray.rows, .stride = gray.cols, .buf = gray.data };
            zarray_t *detections = apriltag_detector_detect(td, &img);

            bool found_now = false;
            for (int i = 0; i < zarray_size(detections); i++) {
                apriltag_detection_t *det;
                zarray_get(detections, i, &det);
                found_now = true;

                // Logic & State Update
                {
                    std::lock_guard<std::mutex> lock(drone_state.mtx);
                    drone_state.last_tag_id = det->id;
                    drone_state.tag_desc = TagAction::get_description(det->id);
                }
                TagAction::execute(det->id);

                // Draw
                for (int j = 0; j < 4; j++)
                    cv::line(frame, cv::Point(det->p[j][0], det->p[j][1]), cv::Point(det->p[(j+1)%4][0], det->p[(j+1)%4][1]), cv::Scalar(0,255,0), 2);
                cv::putText(frame, "ID:"+std::to_string(det->id), cv::Point(det->p[0][0], det->p[0][1]-10), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,0,255), 2);
            }
            if (!found_now) {
                std::lock_guard<std::mutex> lock(drone_state.mtx);
                drone_state.last_tag_id = -1;
            }
            apriltag_detections_destroy(detections);

            std::vector<uchar> buf;
            cv::imencode(".jpg", frame, buf, {cv::IMWRITE_JPEG_QUALITY, 70});
            std::lock_guard<std::mutex> lock(frame_mtx);
            global_frame_buffer = std::move(buf);
        }
    }
    apriltag_detector_destroy(td); tag36h11_destroy(tf);
}

void telemetry_worker() {
    Mavsdk mavsdk{Mavsdk::Configuration{1, 190, false}};
    mavsdk.add_any_connection("serial:///dev/serial0:57600");
    while (is_running && mavsdk.systems().empty()) std::this_thread::sleep_for(std::chrono::seconds(1));
    if (!is_running) return;
    auto telemetry = Telemetry{mavsdk.systems().at(0)};
    telemetry.set_rate_position(2.0); telemetry.set_rate_attitude_euler(5.0);
    telemetry.subscribe_position([](Telemetry::Position p){ std::lock_guard<std::mutex> l(drone_state.mtx); drone_state.alt = p.relative_altitude_m; });
    telemetry.subscribe_attitude_euler([](Telemetry::EulerAngle a){ std::lock_guard<std::mutex> l(drone_state.mtx); drone_state.roll = a.roll_deg; drone_state.pitch = a.pitch_deg; });
    while (is_running) std::this_thread::sleep_for(std::chrono::seconds(1));
}

int main() {
    crow::SimpleApp app;
    CROW_ROUTE(app, "/")([](){
        std::ifstream f("/home/blackbird/Corvus/RaspberryPi/index.html");
        std::stringstream ss; ss << f.rdbuf(); return crow::response(ss.str());
    });
    CROW_ROUTE(app, "/snapshot")([](){
        std::lock_guard<std::mutex> lock(frame_mtx);
        return global_frame_buffer.empty() ? crow::response(404) : crow::response(std::string(global_frame_buffer.begin(), global_frame_buffer.end()));
    });
    CROW_ROUTE(app, "/ws").websocket(&app).onmessage([&](crow::websocket::connection& conn, const std::string&, bool){
        crow::json::wvalue m; { std::lock_guard<std::mutex> l(drone_state.mtx);
            m["alt"]=drone_state.alt; m["roll"]=drone_state.roll; m["pitch"]=drone_state.pitch;
            m["tag_id"]=drone_state.last_tag_id; m["tag_desc"]=drone_state.tag_desc;
        } conn.send_text(m.dump());
    });
    std::thread(camera_worker).detach(); std::thread(telemetry_worker).detach();
    app.port(5000).bindaddr("0.0.0.0").multithreaded().run();
}
