#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include <onnxruntime_cxx_api.h>
#include <memory>
#include <vector>
#include <chrono>
#include <cstdio>
#include <regex>
#include <sstream>
#include <iostream>
#include "ament_index_cpp/get_package_share_directory.hpp" 

using std::placeholders::_1;

class WifiLocalizerNode : public rclcpp::Node {
public:
    WifiLocalizerNode()
    : Node("wifi_localizer_node"), 
    model_path_(ament_index_cpp::get_package_share_directory("wifi_localization_cpp")
                  + "/Net/CNN-net.onnx"),
    session(nullptr), env(ORT_LOGGING_LEVEL_WARNING, "wifi") {

        RCLCPP_INFO(this->get_logger(), "Model path: %s", model_path_.c_str());

        // Declare and get parameters
        this->declare_parameter<std::vector<std::string>>("target_ssids", {"BISTU", "BISTU-802.1X"});

        target_ssids_ = this->get_parameter("target_ssids").as_string_array();

        // Initialize ONNX Session
        Ort::SessionOptions session_options;
        session_options.SetIntraOpNumThreads(1);
        session = Ort::Session(env, model_path_.c_str(), session_options);

        // Setup publishers
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/wifi_pose", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/wifi_path", 10);

        // Timer for periodic WiFi scan and inference
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&WifiLocalizerNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Target SSIDs:");
        for (const auto& ssid : target_ssids_) {
            RCLCPP_INFO(this->get_logger(), " - %s", ssid.c_str());
        }
    }

private:
    void timer_callback() {
        RCLCPP_INFO(this->get_logger(), "Running WiFi scan and inference...");

        std::vector<float> rssi_input = scan_wifi(target_ssids_);
        if (rssi_input.size() < 400) rssi_input.resize(400, 0.0f);
        else if (rssi_input.size() > 400) rssi_input.resize(400);

        std::array<int64_t, 4> input_shape = {1,1,4,100};
        Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            mem_info, rssi_input.data(), rssi_input.size(),
             input_shape.data(), input_shape.size());

        // 修复：正确获取输入输出名称（带索引）
        auto allocator = Ort::AllocatorWithDefaultOptions();
        Ort::AllocatedStringPtr input_name = session.GetInputNameAllocated(0, allocator);
        Ort::AllocatedStringPtr output_name = session.GetOutputNameAllocated(0, allocator);

        // 准备名称指针数组
        const char* input_names[] = {input_name.get()};
        const char* output_names[] = {output_name.get()};

        // 运行推理
        auto output_tensors = session.Run(Ort::RunOptions{nullptr},
            input_names, &input_tensor, 1,
            output_names, 1);

        float* output_data = output_tensors.front().GetTensorMutableData<float>();
        double x = output_data[0];
        double y = output_data[1];

        RCLCPP_INFO(this->get_logger(), "Inferred position -> x: %.2f, y: %.2f", x, y);

        // Publish PoseStamped
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position.x = x;
        pose_msg.pose.position.y = y;
        pose_msg.pose.orientation.w = 1.0;
        pose_pub_->publish(pose_msg);

        // Update and publish path
        path_msg_.header = pose_msg.header;
        path_msg_.poses.push_back(pose_msg);
        path_pub_->publish(path_msg_);
    }

    std::vector<float> scan_wifi(const std::vector<std::string>& target_ssids) {
    // 先全部初始化为 -100
    std::vector<float> rssi_values(target_ssids.size(), -100.0f);

    // 1) 调用 nmcli 取所有可见网络
    FILE* pipe = popen("nmcli -t -f ssid,signal dev wifi", "r");
    if (!pipe) {
        RCLCPP_ERROR(this->get_logger(), "Failed to run nmcli");
        return rssi_values;
    }

    char buffer[256];
    while (fgets(buffer, sizeof(buffer), pipe)) {
        // 每行格式：SSID:signal
        std::string line(buffer);
        // 去掉结尾 '\n'
        if (!line.empty() && line.back() == '\n') line.pop_back();
        // 按冒号拆分
        size_t colon = line.find(':');
        if (colon == std::string::npos) continue;
        std::string ssid = line.substr(0, colon);
        int signal = std::stoi(line.substr(colon + 1));

        // 2) 只记录目标 SSID 的最大信号
        for (size_t i = 0; i < target_ssids.size(); ++i) {
            if (ssid == target_ssids[i]) {
                if (signal > rssi_values[i]) rssi_values[i] = static_cast<float>(signal);
            }
        }
    }
    pclose(pipe);

    // 3) 打印调试
    for (size_t i = 0; i < target_ssids.size(); ++i) {
        RCLCPP_INFO(this->get_logger(), "SSID[%s] -> RSSI %.1f dBm",
                    target_ssids[i].c_str(), rssi_values[i]);
    }

    return rssi_values;
}

    std::string model_path_;
    std::vector<std::string> target_ssids_;
    Ort::Env env;
    Ort::Session session;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_msg_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WifiLocalizerNode>());
    rclcpp::shutdown();
    return 0;
}