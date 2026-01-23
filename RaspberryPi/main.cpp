#include "crow_all.h"
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <vector>
#include <string>
#include <chrono>

// --- Global Data Structure ---
// Using a mutex to prevent "data racing" between the 
// background telemetry thread and the web server threads.
struct DroneState {
    float alt = 0.0;
    int heading = 0;
    float battery = 12.6;
    std::mutex mtx;
} drone_state;

// --- MAVLink Mock Worker ---
// This simulates the Pixhawk data stream. When you get your Pixhawk,
// you will replace the logic inside this while loop with real serial reading.
void telemetry_worker() {
    while (true) {
        {
            std::lock_guard<std::mutex> lock(drone_state.mtx);
            // Simulate realistic flight telemetry
            drone_state.alt += (rand() % 10 - 5) * 0.02f; // Slight drift
            drone_state.heading = (drone_state.heading + 1) % 360; // Rotating
            if (drone_state.battery > 10.5) drone_state.battery -= 0.001f;
        }
        // Update frequency: 10Hz (100ms)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int main() {
    crow::SimpleApp app;

    // --- Camera Setup ---
    // CAP_V4L2 is the hardware-level driver for Linux/Pi
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
                    body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background: #1a1a1a; color: white; text-align: center; }
                    .container { display: flex; flex-direction: column; align-items: center; padding: 20px; }
                    img { border: 4px solid #444; border-radius: 8px; box-shadow: 0 0 20px rgba(0,0,0,0.5); }
                    .telemetry { margin-top: 20px; display: grid; grid-template-columns: repeat(3, 150px); gap: 20px; }
                    .stat { background: #333; padding: 15px; border-radius: 10px; border-bottom: 4px solid #007bff; }
                    .val { font-size: 1.5em; font-weight: bold; color: #007bff; }
                </style>
            </head>
            <body>
                <div class='container'>
                    <h1>LIVE CAMERA FEED</h1>
                    <img src='/video_feed' width='640'>
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
                    // Request update from Pi every 100ms
                    setInterval(() => { if(ws.readyState === 1) ws.send('get'); }, 100);
                </script>
            </body>
            </html>
        )";
    });

    // --- Video Streaming Route ---
    CROW_ROUTE(app, "/video_feed")([&cap]() {
        return crow::response([&cap](crow::response& res) {
            res.set_header("Content-Type", "multipart/x-mixed-replace; boundary=frame");
            while (true) {
                cv::Mat frame;
                cap >> frame;
                if (frame.empty()) break;

                std::vector<uchar> buf;
                // High compression (80) to keep latency low over Wi-Fi
                cv::imencode(".jpg", frame, buf, {cv::IMWRITE_JPEG_QUALITY, 80});
                
                std::string body(buf.begin(), buf.end());
                std::string header = "\r\n--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n";
                
                res.write(header + body);
            }
            res.end();
        });
    });

    // --- WebSocket Route (Telemetry) ---
    CROW_ROUTE(app, "/ws").websocket()
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

    // Start the background thread
    std::thread worker(telemetry_worker);
    worker.detach();

    // Start the server
    app.port(5000).multithreaded().run();
}
