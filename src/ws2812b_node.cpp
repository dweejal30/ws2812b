#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rpi_ws281x/ws2811.h>

#define LED_PIN 18
#define LED_COUNT 30

class WS2812BController : public rclcpp::Node {
public:
    WS2812BController() : Node("ws2812b_controller") {
        // Initialize LED strip
        ws2811.channel[0].gpionum = LED_PIN;
        ws2811.channel[0].count = LED_COUNT;
        ws2811.channel[0].brightness = 255;
        ws2811_init(&ws2811);

        // Subscribe to emotion topic
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "emotion", 10, std::bind(&WS2812BController::onEmotionReceived, this, std::placeholders::_1));
    }

    ~WS2812BController() {
        ws2811_fini(&ws2811);
    }

private:
    void onEmotionReceived(const std_msgs::msg::String::SharedPtr msg) {
        std::string emotion = msg->data;

        if (emotion == "Happy") {
            setColor(0, 255, 0); // Green
        } else if (emotion == "Sad") {
            setColor(0, 0, 255); // Blue
        } else if (emotion == "Cheerful") {
            setColor(255, 255, 0); // Yellow
        } else if (emotion == "Emotional") {
            setColor(255, 0, 255); // Purple
        } else if (emotion == "Excited") {
            setColor(255, 165, 0); // Orange
        } else if (emotion == "Yes") {
            setColor(0, 255, 255); // Cyan
        } else if (emotion == "No") {
            setColor(255, 0, 0); // Red
        }
    }

    void setColor(uint8_t r, uint8_t g, uint8_t b) {
        for (int i = 0; i < LED_COUNT; ++i) {
            ws2811.channel[0].leds[i] = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
        }
        ws2811_render(&ws2811);
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    ws2811_t ws2811 = {};
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WS2812BController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
