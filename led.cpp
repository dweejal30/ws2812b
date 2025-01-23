#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "ws2811.h"

#define LED_COUNT 8          // Number of LEDs in your strip
#define GPIO_PIN 18          // GPIO pin connected to the Data line

class LEDController : public rclcpp::Node {
public:
    LEDController() : Node("led_controller") {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "emotion_topic", 10,
            std::bind(&LEDController::emotion_callback, this, std::placeholders::_1)
        );

        // Initialize ws2811 structure
        led_config.freq = WS2811_TARGET_FREQ;
        led_config.dmanum = 10;
        led_config.channel[0].gpionum = GPIO_PIN;
        led_config.channel[0].count = LED_COUNT;
        led_config.channel[0].invert = 0;
        led_config.channel[0].brightness = 255;
        led_config.channel[0].strip_type = WS2811_STRIP_GRB;

        if (ws2811_init(&led_config) != WS2811_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize WS2811");
            rclcpp::shutdown();
        }
    }

    ~LEDController() {
        ws2811_fini(&led_config);
    }

private:
    void emotion_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string emotion = msg->data;

        uint32_t color = get_color_for_emotion(emotion);
        
        for (int i = 0; i < LED_COUNT; ++i) {
            led_config.channel[0].leds[i] = color;
        }

        ws2811_render(&led_config);
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

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    ws2811_t led_config = {};
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LEDController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

