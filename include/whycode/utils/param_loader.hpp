#ifndef PARAM_LOADER_HPP
#define PARAM_LOADER_HPP

#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace whycon
{

/**
 * @brief YAML-based parameter loader for WhyCon configuration management
 *
 * This class provides an interface for loading and accessing configuration
 * parameters from YAML files.
 *
 * The parameter loader supports:
 * - Hierarchical parameter organization (category/subcategory/parameter)
 * - Type-safe parameter retrieval with automatic conversion
 * - Vector parameter parsing from YAML arrays or delimited strings
 * - Parameter existence checking
 * - Robust error handling with descriptive error messages
 *
 * @example Basic usage:
 * @code
 * ParamLoader loader("config.yaml");
 * int targets = loader.getParams<int>("system", "targets");
 * std::string frame_id = loader.getParams<std::string>("system", "frame_id");
 * double tolerance = loader.getParams<double>("detector", "center_distance_tolerance_ratio");
 * @endcode
 *
 * @example Hierarchical access:
 * @code
 * - Access: ros_interface -> topics -> image_input
 * std::string topic = loader.getParams<std::string>("ros_interface", "topics", "image_input");
 * @endcode
 *
 * @example Vector parameters:
 * @code
 * std::vector<double> thresholds = loader.getParams<double>("detector", "thresholds", ',');
 * @endcode
 */
class ParamLoader
{
public:
    /**
     * @brief Construct a new ParamLoader and load YAML configuration
     *
     * Loads and parses the specified YAML configuration file. Performs validation
     * to ensure the file exists, is readable, and contains valid YAML syntax.
     *
     * @param config_file_path Path to the YAML configuration file
     *
     * @throws std::runtime_error If the file doesn't exist, can't be read, or has invalid YAML syntax
     *
     */
    explicit ParamLoader(const std::string& config_file_path);

    /**
     * @brief Get a parameter value with hierarchical access (category/subcategory/parameter)
     *
     * This template function retrieves parameters from a three-level hierarchy. It supports
     * all basic types (int, float, double, bool, std::string) with automatic type conversion.
     * The function navigates through the YAML structure to locate the specified parameter
     * and converts it to the requested type.
     *
     * @tparam T The expected parameter type (int, float, double, bool, std::string)
     * @param category Top-level category (e.g., "system", "detector", "ros_interface")
     * @param subcategory Second-level subcategory (e.g., "topics", "thresholds")
     * @param parameter Parameter name within the subcategory
     * @return T The parameter value converted to type T
     *
     * @throws std::runtime_error If category, subcategory, or parameter doesn't exist
     * @throws std::runtime_error If type conversion fails
     */
    template <typename T>
    T getParams(
        const std::string& category, const std::string& subcategory,
        const std::string& parameter) const;

    /**
     * @brief Get a parameter value with simple two-level access (category/parameter)
     *
     * This is a convenience overload for direct category->parameter access without subcategories.
     * Internally calls the three-parameter version with an empty subcategory string.
     * Provides a cleaner interface for simple parameter access patterns.
     *
     * @tparam T The expected parameter type (int, float, double, bool, std::string)
     * @param category Top-level category (e.g., "system", "detector")
     * @param parameter Parameter name within the category
     * @return T The parameter value converted to type T
     *
     * @throws std::runtime_error If category or parameter doesn't exist
     * @throws std::runtime_error If type conversion fails
     */
    template <typename T>
    T getParams(const std::string& category, const std::string& parameter) const;

    /**
     * @brief Get a vector parameter from YAML array or delimited string
     *
     * This function can parse vector parameters from two different formats:
     * 1. YAML array format: [value1, value2, value3]
     * 2. Delimited string format: "value1,value2,value3"
     *
     * The function automatically detects the format and parses accordingly.
     * For string format, it performs trimming of whitespace around values.
     *
     * @tparam T Element type for the vector (int, float, double)
     * @param category Top-level category
     * @param parameter Parameter name containing the vector data
     * @param delimiter Character delimiter for string parsing (default: ',')
     * @return std::vector<T> Vector of parsed values
     *
     * @throws std::runtime_error If category or parameter doesn't exist
     * @note Individual element conversion failures are logged but don't stop processing
     */
    template <typename T>
    std::vector<T>
    getParams(const std::string& category, const std::string& parameter, char delimiter) const;

    /**
     * @brief Check if a parameter exists in the configuration
     *
     * Safely checks for parameter existence without throwing exceptions.
     * Uses the navigation system to verify if the specified parameter path is valid.
     *
     * @param category Top-level category
     * @param parameter Parameter name to check
     * @return true if the parameter exists and is accessible, false otherwise
     */
    bool hasParam(const std::string& category, const std::string& parameter) const;

    /**
     * @brief Print the entire configuration to console for debugging
     *
     * Outputs the complete YAML configuration structure to standard output.
     * Useful for debugging configuration issues, understanding parameter hierarchy,
     * or verifying that the configuration was loaded correctly.
     */
    void printAllParams() const;

    /**
     * @brief Load camera intrinsics from camera_intrinsics.yaml file
     *
     * @param camera_file_path Path to camera intrinsics YAML file
     * @param camera_matrix Output 3x3 camera matrix (CV_64F)
     * @param distortion_coeffs Output distortion coefficients (CV_64F)
     * @return true if successful, false on error
     */
    bool loadCameraIntrinsics(
        const std::string& camera_file_path, cv::Mat& camera_matrix,
        cv::Mat& distortion_coeffs) const;

private:
    YAML::Node  config;             ///< Loaded YAML configuration data structure
    std::string config_file_path_;  ///< Path to the configuration file (stored for error reporting)

    /**
     * @brief Navigate to a specific node in the YAML hierarchy (3-level access)
     *
     * Internal method that safely navigates through the YAML structure to find
     * the requested parameter. Handles the three-level hierarchy of category/subcategory/parameter.
     * Provides detailed error reporting if any level in the hierarchy is missing.
     *
     * @param category Top-level category
     * @param subcategory Second-level subcategory
     * @param parameter Parameter name (does not support dot notation in this implementation)
     * @return YAML::Node The located parameter node
     *
     * @throws std::runtime_error If any level in the hierarchy doesn't exist
     */
    YAML::Node navigateNode(
        const std::string& category, const std::string& subcategory,
        const std::string& parameter) const;

    /**
     * @brief Navigate to a specific node in the YAML hierarchy (2-level access)
     *
     * Internal method for direct category->parameter navigation without subcategories.
     * Provides a simpler navigation path for parameters that don't require subcategory organization.
     *
     * @param category Top-level category
     * @param parameter Parameter name (does not support dot notation in this implementation)
     * @return YAML::Node The located parameter node
     *
     * @throws std::runtime_error If category or parameter doesn't exist
     */
    YAML::Node navigateNode(const std::string& category, const std::string& parameter) const;
};

}  // namespace whycon

#endif  // PARAM_LOADER_HPP