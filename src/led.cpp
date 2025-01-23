#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <string>
#include <iostream>

#define LED_COUNT 8 // Number of LEDs in your strip (simulated)

class LEDController : public rclcpp::Node {
public:
    LEDController() : Node("led_controller") {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "emotion_topic", 10,
            std::bind(&LEDController::emotion_callback, this, std::placeholders::_1)
        );

        // Initialize simulated LED array
        leds.resize(LED_COUNT, 0x000000); // Default all LEDs to "off"
        RCLCPP_INFO(this->get_logger(), "LED Controller Node initialized (Simulation Mode)");
    }

private:
    void emotion_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string emotion = msg->data;

        uint32_t color = get_color_for_emotion(emotion);

        // Update simulated LEDs
        for (int i = 0; i < LED_COUNT; ++i) {
            leds[i] = color;
        }

        // Print simulated LED output
        print_leds();
    }

    uint32_t get_color_for_emotion(const std::string &emotion) {
        if (emotion == "Happy") return 0x00FF00;      // Green
        if (emotion == "Sad") return 0x0000FF;        // Blue
        if (emotion == "Cheerful") return 0xFFFF00;   // Yellow
        if (emotion == "Emotional") return 0xFF00FF;  // Purple
        if (emotion == "Excited") return 0xFFA500;    // Orange
        if (emotion == "Yes") return 0xFFFFFF;       // White
        if (emotion == "No") return 0xFF0000;         // Red

        return 0x000000; // Default: Off
    }

    void print_leds() {
        std::cout << "LED Colors: ";
        for (const auto &color : leds) {
            printf("#%06X ", color); // Print color as a hex value
        }
        std::cout << std::endl;
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::vector<uint32_t> leds; // Simulated LED array
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LEDController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
