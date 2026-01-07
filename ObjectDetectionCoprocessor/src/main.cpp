#include "rfdetr_inference.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>

#include <iostream>
#include <cstring>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>

#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/IntegerTopic.h>
#include <wpi/struct/Struct.h>
struct TargetDetection {
    double dx;
    double dy;
    double area;
    double confidence;
    double timestamp;
};

// 2. Manual Template Specialization (Guaranteed to work)
template<>
struct wpi::Struct<TargetDetection> {
    static constexpr std::string_view GetTypeName() { return "struct:TargetDetection"; }
    static constexpr size_t GetSize() { return 40; }
    static constexpr std::string_view GetSchema() { 
        return "double dx; double dy; double area; double confidence; double timestamp"; 
    }

    static TargetDetection Unpack(std::span<const uint8_t> data) {
        return TargetDetection{
            wpi::UnpackStruct<double, 0>(data),
            wpi::UnpackStruct<double, 8>(data),
            wpi::UnpackStruct<double, 16>(data),
            wpi::UnpackStruct<double, 24>(data),
            wpi::UnpackStruct<double, 32>(data)
        };
    }

    static void Pack(std::span<uint8_t> data, const TargetDetection& value) {
        wpi::PackStruct<0>(data, value.dx);
        wpi::PackStruct<8>(data, value.dy);
        wpi::PackStruct<16>(data, value.area);
        wpi::PackStruct<24>(data, value.confidence);
        wpi::PackStruct<32>(data, value.timestamp);
    }
};

// Threaded camera class: capture + undistort + preprocess on a background thread
class ThreadedCamera {
private:
    cv::VideoCapture cap_;
    cv::Mat latest_undistorted_frame_;
    std::vector<float> latest_preprocessed_data_;
    int latest_orig_h_{0}, latest_orig_w_{0};

    std::mutex data_mutex_;
    std::atomic<bool> running_{true};
    std::atomic<bool> new_data_{false};
    std::thread capture_thread_;

    // GPU resources for undistortion
    cv::cuda::GpuMat gpu_map1_, gpu_map2_;
    cv::cuda::GpuMat gpu_frame_, gpu_undistorted_;

    // Reference to inference object for preprocessing
    RFDETRInference* inference_;

public:
    ThreadedCamera(RFDETRInference* inference,
                   const cv::cuda::GpuMat& gpu_map1,
                   const cv::cuda::GpuMat& gpu_map2)
        : inference_(inference) {
        gpu_map1_ = gpu_map1.clone();
        gpu_map2_ = gpu_map2.clone();

        cap_.open(0, cv::CAP_V4L2);
        if (!cap_.isOpened()) {
            throw std::runtime_error("Failed to open camera");
        }

        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
        cap_.set(cv::CAP_PROP_FPS, 30);
        cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);

        capture_thread_ = std::thread(&ThreadedCamera::capture_loop, this);
    }

    ~ThreadedCamera() {
        running_ = false;
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
        cap_.release();
    }

    void capture_loop() {
        cv::Mat frame, undistorted_frame;

        while (running_) {
            if (!cap_.read(frame) || frame.empty()) {
                continue;
            }

            // GPU undistortion
            gpu_frame_.upload(frame);
            cv::cuda::remap(gpu_frame_, gpu_undistorted_,
                            gpu_map1_, gpu_map2_, cv::INTER_LINEAR);
            gpu_undistorted_.download(undistorted_frame);

            // Preprocess using RFDETRInference
            int orig_h, orig_w;
            std::vector<float> preprocessed =
                inference_->preprocess_image(undistorted_frame, orig_h, orig_w);

            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                latest_undistorted_frame_ = undistorted_frame;
                latest_preprocessed_data_ = std::move(preprocessed);
                latest_orig_h_ = orig_h;
                latest_orig_w_ = orig_w;
                new_data_ = true;
            }
        }
    }

    bool get_data(cv::Mat& undistorted_frame,
                  std::vector<float>& preprocessed_data,
                  int& orig_h, int& orig_w) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!new_data_ || latest_undistorted_frame_.empty()) {
            return false;
        }

        undistorted_frame = latest_undistorted_frame_.clone();
        preprocessed_data = latest_preprocessed_data_;
        orig_h = latest_orig_h_;
        orig_w = latest_orig_w_;
        new_data_ = false;
        return true;
    }
};

// Structure to hold object position and orientation data
struct ObjectPosition {
    std::string class_name;
    float distance;        // feet
    float lateral_offset;  // feet (+ = right, - = left)
    float angle_deg;       // signed yaw estimate in degrees
};

ObjectPosition calculate_position(float bottom_center_x, float bottom_y,
                                  const cv::Mat& camera_matrix,
                                  float camera_height_ft = 2.0f,
                                  float camera_pitch_deg = 30.0f) { // New parameter
    
    // Extract intrinsics
    float fx = static_cast<float>(camera_matrix.at<double>(0, 0));
    float fy = static_cast<float>(camera_matrix.at<double>(1, 1));
    float cx = static_cast<float>(camera_matrix.at<double>(0, 2));
    float cy = static_cast<float>(camera_matrix.at<double>(1, 2));

    float camera_height_m = camera_height_ft * 0.3048f;

    // 1. Calculate Normalized Ray in Camera Coordinates
    // (Z is forward, X is right, Y is down in image plane)
    float u_norm = (bottom_center_x - cx) / fx;
    float v_norm = (bottom_y - cy) / fy;

    // 2. Apply Pitch Rotation (Rotation around X-axis)
    // Positive pitch = camera tilting down
    float theta = -camera_pitch_deg * (M_PI / 180.0f);  // note the minus
    float cos_t = std::cos(theta);
    float sin_t = std::sin(theta);

    float ray_x = u_norm;
    float ray_y = v_norm * cos_t - 1.0f * sin_t;
    float ray_z = v_norm * sin_t + 1.0f * cos_t;

    // 3. Intersect with Ground Plane
    // We assume the camera origin is at height H. 
    // The ground is at Y_world = H (relative to camera).
    // Because OpenCV Y is down, the ground is "below" the camera in positive Y.
    // If we want the distance along the floor (Z-forward), we calculate where 
    // the ray hits the "floor" plane defined by the height.
    
    // Protect against dividing by zero (horizon/sky rays)
    if (ray_y < 1e-6f) { 
        // Ray is pointing up or parallel to ground; infinite distance
        ObjectPosition pos;
        pos.distance = -1.0f; // Indicate error/infinity
        pos.lateral_offset = 0.0f;
        pos.angle_deg = 0.0f;
        return pos;
    }

    // Scale factor to reach the ground
    float t = camera_height_m / ray_y;
    float ground_x = ray_x * t;
    float ground_z = ray_z * t;

    // 4. Convert to Output Units
    // float distance_ft = ground_z / 0.3048f;
    // float lateral_offset_ft = ground_x / 0.3048f;

    ObjectPosition pos;
    // Keeping your original unit conversion/scaling logic:
    pos.distance = ground_z;
    pos.lateral_offset = ground_x;
    pos.angle_deg = 0.0f; // You might want atan2(ground_x, ground_z) here

    return pos;

}

// Safe class name lookup with fallback
std::string safe_class_name(const std::vector<std::string>& coco_labels, int class_id) {
    if (class_id >= 0 && class_id < static_cast<int>(coco_labels.size())) {
        return coco_labels[class_id];
    }
    return "class_" + std::to_string(class_id);
}

// Magnitude of coral angle from aspect ratio
float estimate_coral_angle_magnitude(float width, float height) {
    float long_side  = std::max(width, height);
    float short_side = std::max(1.0f, std::min(width, height));
    float aspect = long_side / short_side;

    const float ASPECT_FRONT = 1.56f;
    const float ASPECT_SIDE  = 2.90f;

    float t = (aspect - ASPECT_FRONT) / (ASPECT_SIDE - ASPECT_FRONT);
    t = std::clamp(t, 0.0f, 1.0f);

    return 90.0f * t;
}

// Sign disambiguation using diagonal brightness
float disambiguate_coral_angle_sign(const cv::Mat& frame,
                                    const std::vector<float>& box,
                                    float angle_mag_deg) {
    if (angle_mag_deg < 5.0f) {
        return 0.0f;
    }
    if (angle_mag_deg > 85.0f) {
        return 90.0f;
    }

    int x1 = std::max(0, std::min(frame.cols - 1,
                                  static_cast<int>(std::round(box[0]))));
    int y1 = std::max(0, std::min(frame.rows - 1,
                                  static_cast<int>(std::round(box[1]))));
    int x2 = std::max(0, std::min(frame.cols - 1,
                                  static_cast<int>(std::round(box[2]))));
    int y2 = std::max(0, std::min(frame.rows - 1,
                                  static_cast<int>(std::round(box[3]))));

    if (x2 <= x1 + 2 || y2 <= y1 + 2) {
        return angle_mag_deg;
    }

    cv::Rect roi_rect(x1, y1, x2 - x1, y2 - y1);
    cv::Mat roi = frame(roi_rect);

    cv::Mat gray;
    if (roi.channels() == 3) {
        cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = roi;
    }

    int patch_size = std::max(4, std::min(gray.cols, gray.rows) / 5);

    auto mean_patch = [&](int px, int py) -> double {
        px = std::max(0, std::min(gray.cols - patch_size, px));
        py = std::max(0, std::min(gray.rows - patch_size, py));
        cv::Rect pr(px, py, patch_size, patch_size);
        cv::Scalar m = cv::mean(gray(pr));
        return m[0];
    };

    double tl = mean_patch(0, 0);
    double tr = mean_patch(gray.cols - patch_size, 0);
    double bl = mean_patch(0, gray.rows - patch_size);
    double br = mean_patch(gray.cols - patch_size, gray.rows - patch_size);

    double diag1 = tl + br;
    double diag2 = tr + bl;

    float sign = (diag1 >= diag2) ? 1.0f : -1.0f;

    return angle_mag_deg * sign;
}

// Minimap overlay
void draw_minimap(cv::Mat& frame, const std::vector<ObjectPosition>& positions,
                  const std::vector<int>& class_ids,
                  const std::vector<std::string>& coco_labels) {
    int map_width = 300;
    int map_height = 300;
    int margin = 20;

    cv::Rect map_rect(margin, frame.rows - map_height - margin,
                      map_width, map_height);
    if (map_rect.y < 0) {
        return;
    }

    cv::Mat minimap = cv::Mat::zeros(map_height, map_width, CV_8UC3);
    minimap.setTo(cv::Scalar(40, 40, 40));

    cv::Scalar grid_color(80, 80, 80);
    for (int i = 0; i <= 10; i++) {
        int x = i * map_width / 10;
        cv::line(minimap, cv::Point(x, 0), cv::Point(x, map_height),
                 grid_color, 1);
        int y = i * map_height / 10;
        cv::line(minimap, cv::Point(0, y), cv::Point(map_width, y),
                 grid_color, 1);
    }

    int center_x = map_width / 2;
    int bottom_y = map_height - 10;
    cv::line(minimap, cv::Point(center_x, bottom_y),
             cv::Point(center_x, 0), cv::Scalar(255, 255, 255), 2);
    cv::line(minimap, cv::Point(0, bottom_y),
             cv::Point(map_width, bottom_y), cv::Scalar(255, 255, 255), 2);

    float scale = map_height / 15.0f;  // 15 ft range

    cv::circle(minimap, cv::Point(center_x, bottom_y), 5,
               cv::Scalar(0, 255, 0), -1);
    cv::putText(minimap, "Camera", cv::Point(center_x + 8, bottom_y),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);

    for (size_t i = 0; i < positions.size(); i++) {
        const auto& pos = positions[i];

        int map_x = center_x +
                    static_cast<int>(pos.lateral_offset * scale);
        int map_y = bottom_y -
                    static_cast<int>(pos.distance * scale);

        map_x = std::max(5, std::min(map_width - 5, map_x));
        map_y = std::max(5, std::min(map_height - 5, map_y));

        cv::circle(minimap, cv::Point(map_x, map_y), 4,
                   cv::Scalar(0, 0, 255), -1);

        float line_length = std::min(30.0f, pos.distance * 2.0f);
        float angle_rad = pos.angle_deg * static_cast<float>(M_PI) / 180.0f;

        float dx = std::sin(angle_rad) * line_length * 0.5f;
        float dy = -std::cos(angle_rad) * line_length * 0.5f;

        cv::Point line_start(map_x - static_cast<int>(dx),
                             map_y - static_cast<int>(dy));
        cv::Point line_end(map_x + static_cast<int>(dx),
                           map_y + static_cast<int>(dy));

        cv::line(minimap, line_start, line_end,
                 cv::Scalar(0, 255, 255), 2);

        std::string label = safe_class_name(coco_labels, class_ids[i]);
        std::string distance_str =
            std::to_string(static_cast<int>(pos.distance)) + "ft";

        cv::putText(minimap, label, cv::Point(map_x + 6, map_y - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.35,
                    cv::Scalar(255, 255, 255), 1);
        cv::putText(minimap, distance_str, cv::Point(map_x + 6, map_y + 8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.3,
                    cv::Scalar(200, 200, 200), 1);
    }

    cv::putText(minimap, "15ft", cv::Point(5, 15),
                cv::FONT_HERSHEY_SIMPLEX, 0.4,
                cv::Scalar(255, 255, 255), 1);
    cv::putText(minimap, "0ft", cv::Point(5, map_height - 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.4,
                cv::Scalar(255, 255, 255), 1);

    cv::Mat roi = frame(map_rect);
    cv::addWeighted(roi, 0.3, minimap, 0.7, 0, roi);
}

int main(int argc, char* argv[]) {
    if (argc < 3 || argc > 4) {
        std::cerr << "Usage: " << argv[0]
                  << " <path_to_model> <path_to_coco_labels> [--segmentation]\n";
        return 1;
    }

    const std::filesystem::path model_path = argv[1];
    const std::filesystem::path label_file_path = argv[2];

    bool use_segmentation = false;
    for (int i = 3; i < argc; ++i) {
        if (std::strcmp(argv[i], "--segmentation") == 0) {
            use_segmentation = true;
        }
    }

    try {
        // --- RFDETR inference setup FIRST (before NT or camera) ---
        Config config;
        config.resolution = 0;
        config.model_type = use_segmentation
                                ? ModelType::SEGMENTATION
                                : ModelType::DETECTION;
        config.max_detections = 300;
        config.mask_threshold = 0.0f;

        RFDETRInference inference(model_path, label_file_path, config);
        
        // SAFE: Now coco_labels_ is guaranteed to be populated
        std::cout << "Loaded " << inference.coco_labels_.size() 
                  << " class labels. First: " << inference.coco_labels_[0] << std::endl;

        // --- NetworkTables initialization (Team 6238) ---
        auto nt_inst = nt::NetworkTableInstance::GetDefault();
        nt_inst.StartClient4("Jetson-Coprocessor");
        nt_inst.SetServerTeam(6238);
        nt_inst.StartDSClient();

        auto vision_table = nt_inst.GetTable("ObjectDetection");

        // STRUCT-BASED: Single publisher for all detection data
        auto observations_pub = nt::StructArrayTopic<TargetDetection>(
            vision_table->GetStructArrayTopic<TargetDetection>("observations")
        ).Publish();

        // Keep heartbeat/status publishers
        auto heartbeat_pub = vision_table->GetIntegerTopic("heartbeat").Publish();
        auto connected_pub = vision_table->GetIntegerTopic("nt_connected").Publish();

        std::cout << "UPDATED: NetworkTables initialized for team 6238 (Struct mode2)" << std::endl;

        // Heartbeat counter
        int64_t heartbeat_counter = 0;

        // Camera intrinsics
        cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
            881.7593244433577, 0, 692.1924966278924,
            0, 884.4006847213938, 323.9356429458774,
            0, 0, 1);

        cv::Mat dist_coeffs = (cv::Mat_<double>(5, 1) <<
        0.02448679980406214,
        -0.05520854543496298,
        0.00038178698565343426,
        -0.0012513378998395025,
        0.007390675603541193);

        // Undistortion maps
        cv::Mat map1, map2;
        cv::Size image_size(1280, 720);
        cv::initUndistortRectifyMap(
            camera_matrix, dist_coeffs, cv::Mat(),
            camera_matrix, image_size,
            CV_32FC1, map1, map2);

        cv::cuda::GpuMat gpu_map1, gpu_map2;
        gpu_map1.upload(map1);
        gpu_map2.upload(map2);

        ThreadedCamera camera(&inference, gpu_map1, gpu_map2);

        std::cout << "Camera accessed successfully! Press ESC to quit." << std::endl;

        cv::Mat undistorted_frame;
        std::vector<float> input_data;
        int orig_h = 0, orig_w = 0;

        double fps = 0.0;
        const double fps_alpha = 0.1;

        double avg_fetch_ms = 0.0;
        double avg_inference_ms = 0.0;
        double avg_postprocess_ms = 0.0;
        double avg_drawing_ms = 0.0;

        const int display_scale = 2;

        while (true) {
            auto frame_start = std::chrono::high_resolution_clock::now();
            auto timestamp_us =
                std::chrono::duration_cast<std::chrono::microseconds>(
                    frame_start.time_since_epoch()).count();

            // --- FETCH PREPROCESSED DATA ---
            auto t0 = std::chrono::high_resolution_clock::now();
            if (!camera.get_data(undistorted_frame, input_data,
                                 orig_h, orig_w)) {
                connected_pub.Set(nt_inst.IsConnected() ? 1 : 0);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            auto t1 = std::chrono::high_resolution_clock::now();

            // --- INFERENCE ---
            inference.run_inference(input_data);
            auto t2 = std::chrono::high_resolution_clock::now();

            // --- POST-PROCESS ---
            std::vector<float> scores;
            std::vector<int> class_ids;
            std::vector<std::vector<float>> boxes;
            std::vector<cv::Mat> masks;

            const float scale_w =
                static_cast<float>(orig_w) / inference.get_resolution();
            const float scale_h =
                static_cast<float>(orig_h) / inference.get_resolution();

            if (use_segmentation) {
                inference.postprocess_segmentation_outputs(
                    scale_w, scale_h, orig_h, orig_w,
                    scores, class_ids, boxes, masks);
                inference.draw_segmentation_masks(
                    undistorted_frame, boxes, class_ids, scores, masks);
            } else {
                inference.postprocess_outputs(
                    scale_w, scale_h, scores, class_ids, boxes);
                inference.draw_detections(
                    undistorted_frame, boxes, class_ids, scores);
            }
            auto t3 = std::chrono::high_resolution_clock::now();

            // --- BUILD STRUCT ARRAY + PUBLISH TO NETWORKTABLES ---
            std::vector<TargetDetection> observations;
            observations.reserve(boxes.size());

            std::vector<ObjectPosition> positions;
            positions.reserve(boxes.size());

            for (size_t i = 0; i < boxes.size(); i++) {
                float x1 = boxes[i][0];
                float y1 = boxes[i][1];
                float x2 = boxes[i][2];
                float y2 = boxes[i][3];

                float box_w = std::max(1.0f, x2 - x1);
                float box_h = std::max(1.0f, y2 - y1);
                
                float bottom_center_x = (x1 + x2) * 0.5f;
                float bottom_y = y2;

                if (std::abs(bottom_y - orig_h) <= 20) {
                    continue; // Discard the result if its within the bottom 20 px
                }
                

                // Calculate 3D position`
                ObjectPosition pos = calculate_position(
                    bottom_center_x, bottom_y, camera_matrix);
                pos.class_name = safe_class_name(inference.coco_labels_, class_ids[i]);

                // Calculate angle
                // float angle_mag = estimate_coral_angle_magnitude(box_w, box_h);
                // float angle_signed = disambiguate_coral_angle_sign(
                //     undistorted_frame, boxes[i], angle_mag);
                // pos.angle_deg = angle_signed;

                positions.push_back(pos);

                // Build the struct observation
                // Note: We're using NORMALIZED coordinates (0.0 to 1.0)
                // You can also use pixel coordinates if you prefer - just be consistent with Java
                TargetDetection obs;
                obs.dx = pos.distance;  // Normalized X
                obs.dy = pos.lateral_offset;         // Normalized Y
                obs.area = (box_w * box_h) / static_cast<double>(orig_w * orig_h);
                obs.confidence = scores[i];
                
                // TIMESTAMP SYNC: Apply the NT time offset to match RIO's FPGA clock
                uint64_t local_time_us = nt::Now();  // Local monotonic time in microseconds
                auto offset_opt = nt_inst.GetServerTimeOffset();
                
                if (offset_opt.has_value()) {
                    uint64_t rio_time_us = local_time_us + offset_opt.value();
                    obs.timestamp = rio_time_us / 1000000.0;  // Convert to seconds (like Timer.getFPGATimestamp)
                } else {
                    // Fallback if not connected
                    obs.timestamp = local_time_us / 1000000.0;
                }

                observations.push_back(obs);

                // Draw position text on frame
                std::string pos_text =
                    std::to_string(static_cast<int>(pos.distance)) + "ft, " +
                    std::to_string(static_cast<int>(pos.lateral_offset)) + "ft, " +
                    std::to_string(static_cast<int>(pos.angle_deg)) + "deg";

                cv::putText(undistorted_frame, pos_text,
                            cv::Point(static_cast<int>(x1),
                                    static_cast<int>(y2) + 20),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5,
                            cv::Scalar(255, 255, 0), 2);
            }

            // ATOMIC PUBLISH: All data sent in one NetworkTables update
            observations_pub.Set(observations);

            // Update heartbeat (increments every frame so RIO knows we're alive)
            heartbeat_pub.Set(++heartbeat_counter);
            connected_pub.Set(nt_inst.IsConnected() ? 1 : 0);

            // --- DRAWING ---
            draw_minimap(undistorted_frame, positions, class_ids, inference.coco_labels_);
            auto t4 = std::chrono::high_resolution_clock::now();

            double fetch_ms =
                std::chrono::duration<double, std::milli>(t1 - t0).count();
            double inference_ms =
                std::chrono::duration<double, std::milli>(t2 - t1).count();
            double postprocess_ms =
                std::chrono::duration<double, std::milli>(t3 - t2).count();
            double drawing_ms =
                std::chrono::duration<double, std::milli>(t4 - t3).count();

            static double avg_fetch_ms = 0.0;
            static double avg_inference_ms = 0.0;
            static double avg_postprocess_ms = 0.0;
            static double avg_drawing_ms = 0.0;

            avg_fetch_ms =
                fps_alpha * fetch_ms + (1.0 - fps_alpha) * avg_fetch_ms;
            avg_inference_ms =
                fps_alpha * inference_ms + (1.0 - fps_alpha) * avg_inference_ms;
            avg_postprocess_ms =
                fps_alpha * postprocess_ms + (1.0 - fps_alpha) * avg_postprocess_ms;
            avg_drawing_ms =
                fps_alpha * drawing_ms + (1.0 - fps_alpha) * avg_drawing_ms;

            auto frame_end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = frame_end - frame_start;
            double current_fps = 1.0 / elapsed.count();
            fps = fps_alpha * current_fps + (1.0 - fps_alpha) * fps;

            int text_y = 30;
            int text_x = 10;
            int line_height = 25;

            cv::putText(undistorted_frame,
                        "FPS: " + std::to_string(static_cast<int>(fps)),
                        cv::Point(text_x, text_y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
            text_y += line_height;

            std::string nt_status =
                nt_inst.IsConnected() ? "NT: Connected" : "NT: Disconnected";
            cv::Scalar nt_color =
                nt_inst.IsConnected()
                    ? cv::Scalar(0, 255, 0)
                    : cv::Scalar(0, 0, 255);
            cv::putText(undistorted_frame, nt_status,
                        cv::Point(text_x, text_y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        nt_color, 2, cv::LINE_AA);
            text_y += line_height;

            cv::putText(undistorted_frame, "=== Timing (ms) ===",
                        cv::Point(text_x, text_y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
            text_y += line_height;

            auto draw_timing = [&](const std::string& label, double ms) {
                std::string text = label + ": " +
                    std::to_string(static_cast<int>(ms * 10) / 10.0);
                cv::putText(undistorted_frame, text,
                            cv::Point(text_x, text_y),
                            cv::FONT_HERSHEY_SIMPLEX, 0.45,
                            cv::Scalar(200, 200, 255), 1, cv::LINE_AA);
                text_y += line_height;
            };

            draw_timing("Fetch (threaded)", avg_fetch_ms);
            draw_timing("Inference", avg_inference_ms);
            draw_timing("Postprocess", avg_postprocess_ms);
            draw_timing("Drawing", avg_drawing_ms);

            double total_ms = avg_fetch_ms + avg_inference_ms +
                              avg_postprocess_ms + avg_drawing_ms;
            cv::putText(undistorted_frame,
                        "Total: " +
                        std::to_string(static_cast<int>(total_ms * 10) / 10.0),
                        cv::Point(text_x, text_y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
            text_y += line_height;

            cv::putText(undistorted_frame,
                        "(Capture/Undistort/Preprocess in thread)",
                        cv::Point(text_x, text_y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.35,
                        cv::Scalar(150, 150, 150), 1, cv::LINE_AA);

            cv::Mat display_frame;
            cv::resize(undistorted_frame, display_frame,
                       cv::Size(), display_scale, display_scale,
                       cv::INTER_LINEAR);

            // cv::imshow("RFDETR Live Detection", display_frame);

            char key = static_cast<char>(cv::waitKey(1));
            if (key == 27) {  // ESC
                break;
            }
        }

        cv::destroyAllWindows();

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}

