#define CROW_MAIN
#include "crow_all.h"
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <csignal>

// --- GLOBAL STATE ---
std::atomic<bool> is_running{true};
std::vector<uchar> global_frame_buffer;
std::mutex frame_mtx;

struct DroneState {
    float alt = 10.5;
    int heading = 180;
    std::mutex mtx;
} drone_state;

// Signal handler for Ctrl+C
void signal_handler(int s) {
    is_running = false;
    std::cout << "\nShutting down Corvus GCS..." << std::endl;
    exit(0);
}

// --- CAMERA WORKER: Handles hardware stutters and timeouts ---
void camera_worker() {
    // Pi 5 specific pipeline
    std::string pipeline = "libcamerasrc ! video/x-raw, width=640, height=480, format=NV12 ! videoconvert ! video/x-raw, format=BGR ! appsink drop=true max-buffers=1";
    cv::VideoCapture cap;

    while (is_running) {
        // Attempt to open if not connected
        if (!cap.isOpened()) {
            std::cout << "[CAMERA] Connecting to IMX415..." << std::endl;
            cap.open(pipeline, cv::CAP_GSTREAMER);
            if (!cap.isOpened()) {
                std::this_thread::sleep_for(std::chrono::seconds(2));
                continue;
            }
        }

        cv::Mat frame;
        // Safety check to prevent "Assertion failed" crash
        if (cap.grab()) { 
            if (cap.retrieve(frame) && !frame.empty()) {
                std::vector<uchar> buf;
                cv::imencode(".jpg", frame, buf, {cv::IMWRITE_JPEG_QUALITY, 60});
                
                std::lock_guard<std::mutex> lock(frame_mtx);
                global_frame_buffer = std::move(buf);
            }
        } else {
            // Hardware timeout occurred (like your previous log showed)
            std::cerr << "[CAMERA] Hardware timeout! Re-initializing..." << std::endl;
            cap.release(); 
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
}

// --- TELEMETRY WORKER ---
void telemetry_worker() {
    while (is_running) {
        {
            std::lock_guard<std::mutex> lock(drone_state.mtx);
            drone_state.alt += (rand() % 10 - 5) * 0.05f;
            drone_state.heading = (drone_state.heading + 1) % 360;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int main() {
    std::signal(SIGINT, signal_handler);
    crow::SimpleApp app;

    // --- 1. Main Dashboard UI ---
    CROW_ROUTE(app, "/")([](){
        return R"(
            <!DOCTYPE html>
            <html>
            <head>
                <title>freaking work u idiot</title>
                <style>
                    body { background: #121212; color: white; text-align: center; font-family: sans-serif; margin: 0; padding: 20px; }
                    .card { display: inline-block; background: #1e1e1e; padding: 15px; margin: 10px; border-radius: 8px; border-bottom: 4px solid #007bff; min-width: 120px; }
                    .val { font-size: 1.8em; font-weight: bold; color: #007bff; }
                    img { width: 640px; height: 480px; border: 2px solid #333; background: #000; margin-top: 15px; }
                </style>
            </head>
            <body>
                <h1>CORVUS GROUND CONTROL</h1>
                <div class="card">ALTITUDE<br><span id="alt" class="val">0.0</span> m</div>
                <div class="card">HEADING<br><span id="head" class="val">0</span>&deg;</div>
                <br>
                <img id="stream" src="">
                
                <script>
                    // Snapshot loop: Asks for a frame, waits for it, then asks again
                    const img = document.getElementById('stream');
                    function loadFrame() {
                        img.src = "/snapshot?t=" + new Date().getTime();
                    }
                    img.onload = () => setTimeout(loadFrame, 30); // ~30 FPS
                    img.onerror = () => setTimeout(loadFrame, 1000); // Retry slow if error
                    loadFrame();

                    // Telemetry WebSocket
                    var ws = new WebSocket('ws://' + location.host + '/ws');
                    ws.onmessage = function(v) {
                        var d = JSON.parse(v.data);
                        document.getElementById('alt').innerText = d.alt.toFixed(1);
                        document.getElementById('head').innerText = d.head;
                    };
                    setInterval(() => { if(ws.readyState === 1) ws.send('u'); }, 200);
                </script>
            </body>
            </html>
        )";
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
        res.set_header("Connection", "close");
        return res;
    });

    // --- 3. WebSocket ---
    CROW_ROUTE(app, "/ws").websocket(&app)
        .onmessage([&](crow::websocket::connection& conn, const std::string& data, bool is_binary) {
            crow::json::wvalue msg;
            {
                std::lock_guard<std::mutex> lock(drone_state.mtx);
                msg["alt"] = drone_state.alt;
                msg["head"] = drone_state.heading;
            }
            conn.send_text(msg.dump());
        });

    // Launch background workers
    std::thread cam_thread(camera_worker);
    std::thread tel_thread(telemetry_worker);
    cam_thread.detach();
    tel_thread.detach();

    std::cout << "Corvus GCS active on http://0.0.0.0:5000" << std::endl;
    app.port(5000).bindaddr("0.0.0.0").multithreaded().run();
}
