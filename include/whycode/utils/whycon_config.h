#ifndef WHYCON_CONFIG_H
#define WHYCON_CONFIG_H

#include <ctime>
#include <opencv2/core/hal/intrin.hpp>
#include <string>

// Include the parameter loader
#include "whycode/utils/param_loader.hpp"

/* #undef ENABLE_FULL_UNDISTORT */
/* #undef ENABLE_RANDOMIZED_THRESHOLD */
/* #undef ENABLE_VERBOSE */

// Color codes for terminal output
#define COLOR_RED     "\033[31m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_BLUE    "\033[34m"
#define COLOR_MAGENTA "\033[35m"
#define COLOR_CYAN    "\033[36m"
#define COLOR_WHITE   "\033[37m"
#define COLOR_BOLD    "\033[1m"
#define COLOR_RESET   "\033[0m"

namespace whycon {

/**
 * @brief Global configuration manager for WhyCon system
 *
 * This class provides a centralized way to access all system parameters
 * loaded from YAML configuration files.
 */
class WhyConConfig {
  public:
    /**
     * @brief Initialize configuration from YAML file
     * @param config_file_path Path to YAML configuration file
     */
    static void initialize(const std::string& config_file_path);

    /**
     * @brief Get the parameter loader instance
     * @return Reference to parameter loader
     */
    static const ParamLoader& getParamLoader();

    /**
     * @brief Check if configuration is initialized
     * @return true if initialized, false otherwise
     */
    static bool isInitialized();

  private:
    static std::unique_ptr<ParamLoader> param_loader_;
    static bool                         initialized_;
};
}  // namespace whycon

// Verbose Logging Control (disable for performance analysis)
// #define ENABLE_VERBOSE
#undef ENABLE_VERBOSE

// Log Macros
#if defined(ENABLE_VERBOSE)
#    define WHYCON_DEBUG(x) std::cout << COLOR_YELLOW << "[WHYCON DEBUG] " << x << COLOR_RESET << std::endl
#    define WHYCON_INFO(x)  std::cout << COLOR_GREEN << "[WHYCON INFO] " << x << COLOR_RESET << std::endl
#    define WHYCON_ERROR(x) std::cerr << COLOR_RED << "[WHYCON ERROR] " << x << COLOR_RESET << std::endl
#else
#    define WHYCON_DEBUG(x)
#    define WHYCON_INFO(x)
#    define WHYCON_ERROR(x)
#endif

#endif  // WHYCON_CONFIG_H