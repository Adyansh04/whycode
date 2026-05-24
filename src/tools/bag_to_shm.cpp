#include <cv_bridge/cv_bridge.h>
#include <getopt.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <thread>

#include "iceoryx_posh/popo/untyped_publisher.hpp"
#include "iceoryx_posh/runtime/posh_runtime.hpp"
#include "iox/signal_watcher.hpp"
#include "ros/duration.h"
#include "sensor_msgs/image_encodings.h"

struct RGBImageHeader {
    int      rows;        ///< Image height in pixels
    int      cols;        ///< Image width in pixels
    int      type;        ///< OpenCV image type
    int64_t  timestamp;   ///< Capture timestamp in microseconds
    uint32_t dataOffset;  ///< Offset from payload start to image data
    uint32_t dataSize;    ///< Size of image data in bytes
};

class BagToIceoryx {
  private:
    std::unique_ptr<iox::popo::UntypedPublisher> rgbPublisher_;
    uint32_t                                     rgbFrameCounter_ = 0;

    int64_t rosTimeToMicroseconds(const ros::Time& rosTime) {
        return static_cast<int64_t>(rosTime.sec) * 1000000LL + static_cast<int64_t>(rosTime.nsec) / 1000LL;
    }

  public:
    BagToIceoryx() {
        // Initialize iceoryx runtime
        iox::runtime::PoshRuntime::initRuntime("BagToIceoryx");

        // Create publisher
        iox::capro::ServiceDescription serviceDescription{ "Camera", "Image", "RGB" };
        rgbPublisher_ = std::make_unique<iox::popo::UntypedPublisher>(serviceDescription);
        rgbPublisher_->offer();

        std::cout << "Iceoryx publisher initialized and offering service" << std::endl;
    }

    bool publishRGBImage(const cv::Mat& image, int64_t timestamp) {
        if (!rgbPublisher_ || image.empty()) {
            return false;
        }

        try {
            // Calculate sizes
            size_t dataSize   = image.total() * image.elemSize();
            size_t headerSize = sizeof(RGBImageHeader);
            size_t totalSize  = headerSize + dataSize;

            bool success = false;
            rgbPublisher_->loan(totalSize)
                    .and_then([&](auto& userPayload) {
                        auto* header       = new (userPayload) RGBImageHeader();
                        header->rows       = image.rows;
                        header->cols       = image.cols;
                        header->type       = image.type();
                        header->timestamp  = timestamp;
                        header->dataOffset = headerSize;
                        header->dataSize   = dataSize;

                        // Copy image data after the header
                        uchar* dataPtr = reinterpret_cast<uchar*>(userPayload) + header->dataOffset;
                        std::memcpy(dataPtr, image.data, dataSize);  //! Check

                        rgbPublisher_->publish(userPayload);
                        // std::cout << "Published RGB frame " << rgbFrameCounter_ + 1 << ": " << header->rows << "x"
                        //           << header->cols << " (" << dataSize << " bytes) timestamp: " << timestamp
                        //           << std::endl;
                        success = true;
                    })
                    .or_else([&](auto& error) {
                        std::cerr << "Failed to loan RGB sample: " << error << std::endl;
                        success = false;
                    });

            if (success) {
                rgbFrameCounter_++;
            }
            return success;
        } catch (const std::exception& e) {
            std::cerr << "Exception in publishRGBImage: " << e.what() << std::endl;
            return false;
        }
    }

    bool processBagFile(const std::string& bagPath, const std::string& imageTopic, double playbackRate = 1.0,
                        bool loop = false, double skipSeconds = 0.0) {
        try {
            do {
                rosbag::Bag bag;
                bag.open(bagPath, rosbag::bagmode::Read);

                std::vector<std::string> topics;
                topics.push_back(imageTopic);

                rosbag::View view(bag, rosbag::TopicQuery(topics));

                if (view.size() == 0) {
                    std::cerr << "No messages found for topic: " << imageTopic << std::endl;
                    return false;
                }

                std::cout << "Found " << view.size() << " messages for topic " << imageTopic << std::endl;

                // Calculate bag duration
                ros::Time     bagStartTime = view.getBeginTime();
                ros::Time     bagEndTime   = view.getEndTime();
                ros::Duration bagDuration  = bagEndTime - bagStartTime;

                std::cout << "Bag duration: " << bagDuration.toSec() << " seconds (" << bagDuration.toSec() / 60.0
                          << " minutes)" << std::endl;
                std::cout << "Estimated processing time at " << playbackRate
                          << "x rate: " << bagDuration.toSec() / playbackRate << " seconds" << std::endl;

                // Calculate skip time
                ros::Time skipUntilTime = bagStartTime + ros::Duration(skipSeconds);
                if (skipSeconds > 0.0) {
                    std::cout << "Skipping first " << skipSeconds << " seconds of bag" << std::endl;
                }

                ros::Time lastTimestamp;
                bool      firstMessage  = true;
                uint32_t  skippedFrames = 0;

                for (const rosbag::MessageInstance& m : view) {
                    if (m.getTopic() == imageTopic) {
                        sensor_msgs::Image::ConstPtr imageMsg = m.instantiate<sensor_msgs::Image>();
                        if (imageMsg != nullptr) {
                            // Skip messages before skipUntilTime
                            if (skipSeconds > 0.0 && imageMsg->header.stamp < skipUntilTime) {
                                skippedFrames++;
                                continue;
                            }

                            try {
                                // Convert ROS image to OpenCV - KEEP RGB FORMAT
                                cv_bridge::CvImagePtr cvPtr;
                                if (imageMsg->encoding == "rgb8") {
                                    cvPtr = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::RGB8);
                                } else if (imageMsg->encoding == "bgr8") {
                                    // Convert BGR to RGB for consistency
                                    cvPtr = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::RGB8);
                                } else if (imageMsg->encoding == "mono8") {
                                    cvPtr = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::MONO8);
                                } else {
                                    // Convert any other format to RGB8
                                    std::cout << "Converting from " << imageMsg->encoding << " to RGB8" << std::endl;
                                    cvPtr = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::RGB8);
                                }

                                int64_t timestamp = rosTimeToMicroseconds(imageMsg->header.stamp);

                                // Publish to iceoryx
                                if (!publishRGBImage(cvPtr->image, timestamp)) {
                                    std::cerr << "Failed to publish image" << std::endl;
                                    continue;
                                }

                                // Control playback rate
                                if (!firstMessage && playbackRate > 0) {
                                    ros::Duration timeDiff  = imageMsg->header.stamp - lastTimestamp;
                                    double        sleepTime = timeDiff.toSec() / playbackRate;
                                    if (sleepTime > 0) {
                                        std::this_thread::sleep_for(std::chrono::duration<double>(sleepTime));
                                    }
                                }

                                lastTimestamp = imageMsg->header.stamp;
                                firstMessage  = false;

                            } catch (cv_bridge::Exception& e) {
                                std::cerr << "cv_bridge exception: " << e.what() << std::endl;
                                continue;
                            }
                        }
                    }
                }

                bag.close();

                if (skipSeconds > 0.0) {
                    std::cout << "Skipped " << skippedFrames << " frames in first " << skipSeconds << " seconds"
                              << std::endl;
                }

                std::cout << "Finished processing bag file. Published " << rgbFrameCounter_ << " frames total."
                          << std::endl;

                if (loop) {
                    std::cout << "Looping back to start of bag..." << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }

            } while (loop);

            return true;

        } catch (rosbag::BagException& e) {
            std::cerr << "Error opening bag file: " << e.what() << std::endl;
            return false;
        }
    }
};

void printUsage(const char* programName) {
    std::cerr << "Usage: " << programName << " <bag_file_path> <image_topic> [options]\n"
              << "Options:\n"
              << "  -r <rate>     Playback rate multiplier (default: 1.0)\n"
              << "  -l            Loop playback indefinitely\n"
              << "  -s <seconds>  Skip first X seconds of bag\n"
              << "  -h            Show this help message\n"
              << "\nExample: " << programName << " data.bag /camera/image_raw -r 2.0 -l -s 10.5\n";
}

int main(int argc, char** argv) {
    if (argc < 3) {
        printUsage(argv[0]);
        return -1;
    }

    std::string bagPath      = argv[1];
    std::string imageTopic   = argv[2];
    double      playbackRate = 1.0;
    bool        loop         = false;
    double      skipSeconds  = 0.0;

    // Parse command line options
    int opt;
    optind = 3;  // Start parsing from the 3rd argument
    while ((opt = getopt(argc, argv, "r:ls:h")) != -1) {
        switch (opt) {
        case 'r':
            playbackRate = std::atof(optarg);
            if (playbackRate <= 0) {
                std::cerr << "Error: Playback rate must be positive" << std::endl;
                return -1;
            }
            break;
        case 'l':
            loop = true;
            break;
        case 's':
            skipSeconds = std::atof(optarg);
            if (skipSeconds < 0) {
                std::cerr << "Error: Skip seconds must be non-negative" << std::endl;
                return -1;
            }
            break;
        case 'h':
            printUsage(argv[0]);
            return 0;
        case '?':
            printUsage(argv[0]);
            return -1;
        }
    }

    std::cout << "Starting bag to iceoryx converter..." << std::endl;
    std::cout << "Bag file: " << bagPath << std::endl;
    std::cout << "Image topic: " << imageTopic << std::endl;
    std::cout << "Playback rate: " << playbackRate << "x" << std::endl;
    std::cout << "Loop: " << (loop ? "enabled" : "disabled") << std::endl;
    std::cout << "Skip first: " << skipSeconds << " seconds" << std::endl;

    BagToIceoryx converter;

    // Small delay to allow subscriber to connect
    std::this_thread::sleep_for(std::chrono::seconds(2));

    if (!converter.processBagFile(bagPath, imageTopic, playbackRate, loop, skipSeconds)) {
        std::cerr << "Failed to process bag file" << std::endl;
        return -1;
    }

    // Keep running for a bit to ensure all data is sent
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Conversion complete!" << std::endl;

    return 0;
}