#include <csignal>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "iceoryx_posh/popo/listener.hpp"
#include "iceoryx_posh/popo/untyped_subscriber.hpp"
#include "iceoryx_posh/runtime/posh_runtime.hpp"
#include "iox/signal_watcher.hpp"

struct RGBImageHeader {
    int      rows;        ///< Image height in pixels
    int      cols;        ///< Image width in pixels
    int      type;        ///< OpenCV image type
    int64_t  timestamp;   ///< Capture timestamp in microseconds
    uint32_t dataOffset;  ///< Offset from payload start to image data
    uint32_t dataSize;    ///< Size of image data in bytes
};

constexpr char APP_NAME[] = "iox-cpp-image-subscriber-listener";

class IceoryxImageSubscriber {
  public:
    IceoryxImageSubscriber() : rgbSubscriber_({ "Camera", "Image", "RGB" }) {
        rgbSubscriber_.subscribe();

        // Attach event with callback
        listener_
                .attachEvent(rgbSubscriber_, iox::popo::SubscriberEvent::DATA_RECEIVED,
                             iox::popo::createNotificationCallback(onSampleReceivedCallback, *this))
                .or_else([](auto) {
                    std::cerr << "unable to attach subscriber event" << std::endl;
                    std::exit(EXIT_FAILURE);
                });

        if (displayImages_) {
            cv::namedWindow("RGB Client - Received Image", cv::WINDOW_AUTOSIZE);
        }

        std::cout << "Iceoryx subscriber initialized with listener callbacks" << std::endl;
    }

    ~IceoryxImageSubscriber() {
        if (displayImages_) {
            cv::destroyAllWindows();
        }
    }

    void waitForShutdown() {
        std::cout << "Starting listener-based image subscriber..." << std::endl;
        std::cout << "Press Ctrl+C to exit" << std::endl;

        iox::waitForTerminationRequest();
        std::cout << "Shutdown requested, exiting gracefully." << std::endl;
    }

    void printStats() const {
        std::cout << "\n=== Statistics ===" << std::endl;
        std::cout << "Total frames received: " << frameCounter_ << std::endl;
        std::cout << "Dropped frames: " << droppedFrames_ << std::endl;
    }

  private:
    static void onSampleReceivedCallback(iox::popo::UntypedSubscriber* subscriber, IceoryxImageSubscriber* self) {
        int processedInCallback = 0;

        // Process ALL available samples to prevent backlog
        while (subscriber->take()
                       .and_then([subscriber, self, &processedInCallback](const void* userPayload) {
                           processedInCallback++;

                           if (self->verbose_ && !self->firstFrameReceived_) {
                               std::cout << "First frame received!" << std::endl;
                           }
                           self->firstFrameReceived_ = true;

                           auto* header = static_cast<const RGBImageHeader*>(userPayload);

                           uchar* dataPtr =
                                   reinterpret_cast<uchar*>(const_cast<RGBImageHeader*>(header)) + header->dataOffset;

                           cv::Mat rgbImage(header->rows, header->cols, header->type);
                           std::memcpy(rgbImage.data, dataPtr, header->dataSize);

                           if (!rgbImage.empty() && self->displayImages_) {
                               // Convert RGB to BGR for proper OpenCV display
                               cv::Mat bgrImage;
                               if (rgbImage.channels() == 3) {
                                   cv::cvtColor(rgbImage, bgrImage, cv::COLOR_RGB2BGR);
                                   cv::imshow("RGB Client - Received Image", bgrImage);
                               } else {
                                   // Mono image - display as-is
                                   cv::imshow("RGB Client - Received Image", rgbImage);
                               }
                               cv::waitKey(1);
                           }

                           self->frameCounter_++;

                           // Explicit release for faster cleanup
                           subscriber->release(userPayload);
                       })
                       .has_error()) {
            // Break on error or no more chunks
            break;
        }

        if (processedInCallback > 1 && self->verbose_) {
            std::cout << "Processed " << processedInCallback << " samples in callback" << std::endl;
        }
    }

    // Member variables
    iox::popo::UntypedSubscriber rgbSubscriber_;
    iox::popo::Listener          listener_;

    uint32_t frameCounter_       = 0;
    uint32_t droppedFrames_      = 0;
    bool     firstFrameReceived_ = false;
    bool     displayImages_      = true;
    bool     verbose_            = false;  // Reduce logging
    int      processingSkip_     = 1;      // Process every frame, increase to skip frames
};

int main() {
    std::cout << "Starting iceoryx image subscriber with Listener callbacks..." << std::endl;

    try {
        iox::runtime::PoshRuntime::initRuntime(APP_NAME);
        IceoryxImageSubscriber subscriber;
        subscriber.waitForShutdown();
        subscriber.printStats();

    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
