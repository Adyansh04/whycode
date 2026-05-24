#include "whycode/image/debug_image_manager.hpp"

#include <algorithm>
#include <cmath>

namespace whycon {

DebugImageManager::DebugImageManager(bool enabled) : enabled_(enabled) {
    // Reserve space for typical debug image count
    debug_images_.reserve(8);
    image_titles_.reserve(8);
}

void DebugImageManager::addDebugImage(const cv::Mat& image, const std::string& title) {
    if (!enabled_ || image.empty()) {
        return;
    }

    // Store copy of image for later processing
    debug_images_.push_back(image.clone());
    image_titles_.push_back(title);
}

void DebugImageManager::clearDebugImages() {
    debug_images_.clear();
    image_titles_.clear();
}

cv::Mat DebugImageManager::createConsolidatedImage() {
    if (!enabled_ || debug_images_.empty()) {
        return cv::Mat();
    }

    int        num_images  = debug_images_.size();
    GridLayout grid_layout = calculateGridLayout(num_images);
    int        grid_rows   = grid_layout.rows;
    int        grid_cols   = grid_layout.cols;

    // Calculate cell dimensions (aiming for reasonable resolution)
    const int max_total_width  = 1920;  // Max consolidated image width
    const int max_total_height = 1080;  // Max consolidated image height

    int cell_width  = max_total_width / grid_cols;
    int cell_height = max_total_height / grid_rows;

    // Create consolidated image
    cv::Mat consolidated_image = cv::Mat::zeros(grid_rows * cell_height, grid_cols * cell_width, CV_8UC3);

    cv::Mat resized_image;  // Declare reusable Mat
    for (int i = 0; i < num_images; i++) {
        int row = i / grid_cols;
        int col = i % grid_cols;

        // Resize debug image to fit cell
        resizeToFit(debug_images_[i], resized_image, cell_width, cell_height);

        // Calculate position in consolidated image
        int y_offset = row * cell_height;
        int x_offset = col * cell_width;

        // Ensure the image fits within bounds
        int actual_height = std::min(resized_image.rows, cell_height);
        int actual_width  = std::min(resized_image.cols, cell_width);

        // Copy resized image to consolidated image
        cv::Rect roi(x_offset, y_offset, actual_width, actual_height);
        resized_image(cv::Rect(0, 0, actual_width, actual_height)).copyTo(consolidated_image(roi));

        // Add title text
        if (!image_titles_[i].empty()) {
            cv::putText(consolidated_image, image_titles_[i], cv::Point(x_offset + 5, y_offset + 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        }

        // Draw border around each image
        cv::rectangle(consolidated_image, cv::Point(x_offset, y_offset),
                      cv::Point(x_offset + cell_width - 1, y_offset + cell_height - 1), cv::Scalar(128, 128, 128), 1);
    }

    return consolidated_image;
}

GridLayout DebugImageManager::calculateGridLayout(int num_images) {
    if (num_images <= 0)
        return GridLayout(1, 1);
    if (num_images == 1)
        return GridLayout(1, 1);
    if (num_images <= 2)
        return GridLayout(1, 2);
    if (num_images <= 4)
        return GridLayout(2, 2);
    if (num_images <= 6)
        return GridLayout(2, 3);
    if (num_images <= 9)
        return GridLayout(3, 3);
    if (num_images <= 12)
        return GridLayout(3, 4);

    // For larger numbers, calculate approximately square grid
    int cols = static_cast<int>(std::ceil(std::sqrt(num_images)));
    int rows = static_cast<int>(std::ceil(static_cast<double>(num_images) / cols));
    return GridLayout(rows, cols);
}

void DebugImageManager::resizeToFit(const cv::Mat& image, cv::Mat& output, int target_width, int target_height) {
    if (image.empty()) {
        output = cv::Mat::zeros(target_height, target_width, CV_8UC3);
        return;
    }

    // Convert to 3-channel if needed
    cv::Mat color_image;
    if (image.channels() == 1) {
        cv::cvtColor(image, color_image, cv::COLOR_GRAY2BGR);
    } else if (image.channels() == 3) {
        color_image = image;
    } else {
        cv::cvtColor(image, color_image, cv::COLOR_RGBA2BGR);
    }

    // Calculate scaling to maintain aspect ratio
    double scale_x = static_cast<double>(target_width) / color_image.cols;
    double scale_y = static_cast<double>(target_height) / color_image.rows;
    double scale   = std::min(scale_x, scale_y);

    int new_width  = static_cast<int>(color_image.cols * scale);
    int new_height = static_cast<int>(color_image.rows * scale);

    cv::Mat resized;
    cv::resize(color_image, resized, cv::Size(new_width, new_height));

    // Create final image with padding if needed
    output       = cv::Mat::zeros(target_height, target_width, CV_8UC3);
    int x_offset = (target_width - new_width) / 2;
    int y_offset = (target_height - new_height) / 2;

    resized.copyTo(output(cv::Rect(x_offset, y_offset, new_width, new_height)));
}

}  // namespace whycon