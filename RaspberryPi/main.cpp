#define CROW_MAIN
#include "crow_all.h"
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <vector>
#include <string>
#include <chrono>

// --- Global Data Structure ---
struct DroneState {
    float alt = 0.0;
    int heading = 0;
    float battery = 12.6;
    std::mutex mtx;
} drone_state;

// --- MAVLink Mock Worker ---
void telemetry_worker() {
    while (true) {
        {
            std::lock_guard<std::mutex> lock(drone_state.mtx);
            // Simulate realistic flight telemetry
            drone_state.alt += (rand() % 10 - 5) * 0.02f; 
            drone_state.heading = (drone_state.heading + 1) % 360; 
            if (drone_state.battery > 10.5) drone_state.battery -= 0.001f;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int main() {
    crow::SimpleApp app;

    // --- Camera Setup ---
    cv::VideoCapture cap(0, cv::CAP_V4L2);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 30);

    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera!" << std::endl;
        return -1;
    }

    // --- Main Route (Frontend) ---
    CROW_ROUTE(app, "/")([](){
        return R"(
            <!DOCTYPE html>
            <html>
            <head>
                <title>Pi-Pixhawk GCS</title>
                <style>
                    body { font-family: sans-serif; background: #1a1a1a; color: white; text-align: center; }
                    .container { display: flex; flex-direction: column; align-items: center; padding: 20px; }
                    img { border: 4px solid #444; border-radius: 8px; width: 640px; background: #000; }
                    .telemetry { margin-top: 20px; display: grid; grid-template-columns: repeat(3, 150px); gap: 20px; }
                    .stat { background: #333; padding: 15px; border-radius: 10px; border-bottom: 4px solid #007bff; }
                    .val { font-size: 1.5em; font-weight: bold; color: #007bff; }
                </style>
            </head>
            <body>
                <div class='container'>
                    <h1>CORVUS LIVE GCS</h1>
                    <img src='/video_feed'>
                    <div class='telemetry'>
                        <div class='stat'>ALTITUDE<br><span id='alt' class='val'>0</span> m</div>
                        <div class='stat'>HEADING<br><span id='head' class='val'>0</span>&deg;</div>
                        <div class='stat'>BATTERY<br><span id='batt' class='val'>0</span> V</div>
                    </div>
                </div>
                <script>
                    var ws = new WebSocket('ws://' + location.host + '/ws');
                    ws.onmessage = function(event) {
                        var data = JSON.parse(event.data);
                        document.getElementById('alt').innerText = data.alt.toFixed(2);
                        document.getElementById('head').innerText = data.heading;
                        document.getElementById('batt').innerText = data.batt.toFixed(2);
                    };
                    setInterval(() => { if(ws.readyState === 1) ws.send('get'); }, 100);
                </script>
            </body>
            </html>
        )";
    });

    // --- Video Streaming Route (Corrected for Crow 1.1.0) ---
    CROW_ROUTE(app, "/video_feed")([&cap](const crow::request& req, crow::response& res) {
        res.set_header("Content-Type", "multipart/x-mixed-replace; boundary=frame");
        while (true) {
            cv::Mat frame;
            cap >> frame;
            if (frame.empty()) break;

            std::vector<uchar> buf;
            cv::imencode(".jpg", frame, buf, {cv::IMWRITE_JPEG_QUALITY, 70});
            
            std::string body(buf.begin(), buf.end());
            std::string header = "\r\n--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
            
            res.write(header + body);
            
            // Short sleep to prevent CPU Max-out
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }
        res.end();
    });

    // --- WebSocket Route (Corrected with &app pointer) ---
    CROW_ROUTE(app, "/ws").websocket(&app)
        .onmessage([&](crow::websocket::connection& conn, const std::string& data, bool is_binary) {
            crow::json::wvalue msg;
            {
                std::lock_guard<std::mutex> lock(drone_state.mtx);
                msg["alt"] = drone_state.alt;
                msg["heading"] = drone_state.heading;
                msg["batt"] = drone_state.battery;
            }
            conn.send_text(msg.dump());
        });

    // Start threads and server
    std::thread worker(telemetry_worker);
    worker.detach();

    std::cout << "GCS Server starting on http://0.0.0.0:5000" << std::endl;
    app.port(5000).multithreaded().run();
}
