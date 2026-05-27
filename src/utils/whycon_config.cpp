#include "whycode/utils/whycon_config.h"

#include <iostream>

namespace whycon {

std::unique_ptr<ParamLoader> WhyConConfig::param_loader_ = nullptr;
bool                         WhyConConfig::initialized_  = false;

void WhyConConfig::initialize(const std::string& config_file_path) {
    try {
        param_loader_ = std::make_unique<ParamLoader>(config_file_path);
        initialized_  = true;

        std::cout << "[WhyConConfig] Configuration initialized successfully" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "[WhyConConfig] Failed to initialize configuration: " << e.what() << std::endl;
        throw;
    }
}

const ParamLoader& WhyConConfig::getParamLoader() {
    if (!initialized_ || !param_loader_) {
        throw std::runtime_error("WhyConConfig not initialized. Call initialize() first.");
    }
    return *param_loader_;
}

bool WhyConConfig::isInitialized() {
    return initialized_;
}

}  // namespace whycon