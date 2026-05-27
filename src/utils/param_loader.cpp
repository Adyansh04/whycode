#include "whycode/utils/param_loader.hpp"

#include <fstream>

namespace whycon
{

ParamLoader::ParamLoader(const std::string& config_file_path)
  : config_file_path_(config_file_path)
{
    try
    {
        // File exists and is readable
        std::ifstream file(config_file_path);
        if (!file.good())
        {
            throw std::runtime_error("Configuration file not found: " + config_file_path);
        }
        file.close();

        // Load and parse the YAML configuration file
        config = YAML::LoadFile(config_file_path);

        // Validate that the loaded YAML is not null or empty
        if (config.IsNull())
        {
            throw std::runtime_error("Failed to load YAML configuration from: " + config_file_path);
        }

        std::cout << "[ParamLoader] Successfully loaded configuration from: " << config_file_path
                  << std::endl;
    }
    catch (const YAML::Exception& e)
    {
        throw std::runtime_error("YAML parsing error: " + std::string(e.what()));
    }
}

YAML::Node ParamLoader::navigateNode(
    const std::string& category, const std::string& subcategory, const std::string& parameter) const
{
    // Validate that the top-level category exists
    if (!config[category])
    {
        throw std::runtime_error("Category not found: " + category);
    }

    // Start navigation from the category level
    YAML::Node node = config[category];

    // This handles the second level: config[category][subcategory]
    if (!subcategory.empty())
    {
        if (!node[subcategory])
        {
            throw std::runtime_error("Subcategory not found: " + category + "/" + subcategory);
        }
        node = node[subcategory];
    }

    if (!node[parameter])
    {
        std::cerr << "[ParamLoader] Parameter not found: " << category << "/" << subcategory << "/"
                  << parameter << std::endl;
        throw std::runtime_error("Parameter not found: " + parameter);
    }

    return node[parameter];
}

YAML::Node ParamLoader::navigateNode(const std::string& category, const std::string& parameter) const
{
    // Validate that the top-level category exists in the configuration
    if (!config[category])
    {
        throw std::runtime_error("Category not found: " + category);
    }

    // Start navigation from the category level
    YAML::Node node = config[category];

    if (!node[parameter])
    {
        std::cerr << "[ParamLoader] Parameter not found: " << category << "/" << parameter
                  << std::endl;
        throw std::runtime_error("Parameter not found: " + parameter);
    }

    return node[parameter];
}

template <typename T>
T ParamLoader::getParams(
    const std::string& category, const std::string& subcategory, const std::string& parameter) const
{
    try
    {
        YAML::Node node;

        // Choose the appropriate navigation method based on whether subcategory is provided
        if (subcategory.empty())
        {
            // Use two-level navigation: category -> parameter
            node = navigateNode(category, parameter);
        }
        else
        {
            // Use three-level navigation: category -> subcategory -> parameter
            node = navigateNode(category, subcategory, parameter);
        }

        std::cout << "[ParamLoader] Successfully loaded " << category << "/" << parameter << ": "
                  << node.as<T>() << std::endl;

        // Convert the YAML node to the requested type using yaml-cpp's automatic conversion
        return node.as<T>();
    }
    catch (const YAML::BadConversion& e)
    {
        std::cerr << "[ParamLoader] Bad conversion for parameter " << category << "/" << subcategory
                  << "/" << parameter << ": " << e.what() << std::endl;
        throw std::runtime_error("Parameter conversion failed");
    }
    catch (const std::exception& e)
    {
        std::cerr << "[ParamLoader] Using default value for " << category << "/" << parameter
                  << std::endl;
        throw;
    }
}

template <typename T>
T ParamLoader::getParams(const std::string& category, const std::string& parameter) const
{
    // Delegate to the three-parameter version with empty subcategory
    return getParams<T>(category, "", parameter);
}

template <typename T>
std::vector<T> ParamLoader::getParams(
    const std::string& category, const std::string& parameter, char delimiter) const
{
    try
    {
        // Navigate to the parameter location in the YAML structure
        YAML::Node node = navigateNode(category, parameter);

        // Handle YAML sequence/array format: [value1, value2, value3]
        if (node.IsSequence())
        {
            std::vector<T> values;
            for (const auto& item : node)
            {
                values.push_back(item.as<T>());
            }
            return values;
        }

        // Handle comma-separated string format: "value1,value2,value3"
        std::string        param_value = node.as<std::string>();
        std::vector<T>     values;
        std::istringstream iss(param_value);
        std::string        token;

        // Parse the delimited string token by token
        while (std::getline(iss, token, delimiter))
        {
            // Handles spaces around delimiters: "value1, value2, value3"
            token.erase(0, token.find_first_not_of(" \t"));
            token.erase(token.find_last_not_of(" \t") + 1);

            std::stringstream convert(token);
            T                 value;
            if (!(convert >> value))
            {
                std::cerr << "[ParamLoader] Conversion failed for token: " << token << std::endl;
                continue;
            }
            values.push_back(value);
        }
        return values;
    }
    catch (const YAML::BadConversion& e)
    {
        std::cerr << "[ParamLoader] Bad conversion for vector parameter " << category << "/"
                  << parameter << ": " << e.what() << std::endl;
        return {};  // Return empty vector on failure
    }
}

bool ParamLoader::hasParam(const std::string& category, const std::string& parameter) const
{
    try
    {
        // Attempt to navigate to the parameter - if successful, it exists
        navigateNode(category, parameter);
        return true;
    }
    catch (const std::exception&)
    {
        return false;
    }
}

void ParamLoader::printAllParams() const
{
    std::cout << "[ParamLoader] Configuration from: " << config_file_path_ << std::endl;
    std::cout << config << std::endl;
}

bool ParamLoader::loadCameraIntrinsics(
    const std::string& camera_file_path, cv::Mat& camera_matrix, cv::Mat& distortion_coeffs) const
{
    try
    {
        // Load camera intrinsics YAML file
        YAML::Node camera_config = YAML::LoadFile(camera_file_path);

        // Load camera matrix
        if (camera_config["camera_matrix"] && camera_config["camera_matrix"]["data"])
        {
            std::vector<double> camera_data =
                camera_config["camera_matrix"]["data"].as<std::vector<double>>();
            if (camera_data.size() == 9)
            {
                camera_matrix = cv::Mat(3, 3, CV_64F);
                for (int i = 0; i < 9; i++)
                {
                    camera_matrix.at<double>(i / 3, i % 3) = camera_data[i];
                }
            }
            else
            {
                std::cerr << "[ParamLoader] Invalid camera matrix size: " << camera_data.size()
                          << std::endl;
                return false;
            }
        }
        else
        {
            std::cerr << "[ParamLoader] Camera matrix not found in " << camera_file_path
                      << std::endl;
            return false;
        }

        // Load distortion coefficients
        if (camera_config["distortion_coefficients"] &&
            camera_config["distortion_coefficients"]["data"])
        {
            std::vector<double> dist_data =
                camera_config["distortion_coefficients"]["data"].as<std::vector<double>>();
            if (dist_data.size() >= 4 && dist_data.size() <= 8)
            {
                distortion_coeffs = cv::Mat(1, dist_data.size(), CV_64F);
                for (size_t i = 0; i < dist_data.size(); i++)
                {
                    distortion_coeffs.at<double>(0, i) = dist_data[i];
                }
            }
            else
            {
                std::cerr << "[ParamLoader] Invalid distortion coefficients size: "
                          << dist_data.size() << std::endl;
                return false;
            }
        }
        else
        {
            std::cerr << "[ParamLoader] Distortion coefficients not found in " << camera_file_path
                      << std::endl;
            return false;
        }

        std::cout << "[ParamLoader] Successfully loaded camera intrinsics from: "
                  << camera_file_path << std::endl;
        return true;
    }
    catch (const YAML::Exception& e)
    {
        std::cerr << "[ParamLoader] YAML error loading camera intrinsics: " << e.what()
                  << std::endl;
        return false;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[ParamLoader] Error loading camera intrinsics: " << e.what() << std::endl;
        return false;
    }
}

// Three-parameter version (category, subcategory, parameter) - ALL TYPES INCLUDING STRING
template std::string ParamLoader::getParams<std::string>(
    const std::string&, const std::string&, const std::string&) const;
template int
ParamLoader::getParams<int>(const std::string&, const std::string&, const std::string&) const;
template float
ParamLoader::getParams<float>(const std::string&, const std::string&, const std::string&) const;
template double
ParamLoader::getParams<double>(const std::string&, const std::string&, const std::string&) const;
template bool
ParamLoader::getParams<bool>(const std::string&, const std::string&, const std::string&) const;

// Two-parameter version (category, parameter) - ALL TYPES INCLUDING STRING
template std::string
                ParamLoader::getParams<std::string>(const std::string&, const std::string&) const;
template int    ParamLoader::getParams<int>(const std::string&, const std::string&) const;
template float  ParamLoader::getParams<float>(const std::string&, const std::string&) const;
template double ParamLoader::getParams<double>(const std::string&, const std::string&) const;
template bool   ParamLoader::getParams<bool>(const std::string&, const std::string&) const;

// Vector parameter instantiations for common numeric types
template std::vector<double>
ParamLoader::getParams<double>(const std::string&, const std::string&, char) const;
template std::vector<float>
ParamLoader::getParams<float>(const std::string&, const std::string&, char) const;
template std::vector<int>
ParamLoader::getParams<int>(const std::string&, const std::string&, char) const;

}  // namespace whycon