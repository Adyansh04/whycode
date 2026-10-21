#ifndef WHYCON_RECYCLED_IMAGE_STRATEGY_HPP
#define WHYCON_RECYCLED_IMAGE_STRATEGY_HPP

#include <memory>
#include <rclcpp/message_memory_strategy.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace whycon
{
/**
 * @brief Message memory for the camera subscription that reuses the last frame's message.
 *
 * rclcpp's default allocates a new Image for every frame, and deserialisation then zero-fills its
 * 2.7 MB buffer before copying the frame in. Once nothing holds the last frame's message, it is
 * handed back instead: its buffer already has the frame's size and is only overwritten. Takes on
 * the subscription must not run concurrently, as in its default mutually exclusive callback group.
 */
class RecycledImageStrategy
  : public rclcpp::message_memory_strategy::MessageMemoryStrategy<sensor_msgs::msg::Image>
{
public:
    /**
     * @brief The message to deserialise the next frame into.
     * @return The last frame's message if nothing else holds it, otherwise a new one.
     */
    std::shared_ptr<sensor_msgs::msg::Image> borrow_message() override
    {
        if (!message_ || message_.use_count() > 1)
        {
            message_ = std::make_shared<sensor_msgs::msg::Image>();
        }
        return message_;
    }

private:
    std::shared_ptr<sensor_msgs::msg::Image> message_;  // The last frame's message, kept for reuse
};
}  // namespace whycon

#endif  // WHYCON_RECYCLED_IMAGE_STRATEGY_HPP
