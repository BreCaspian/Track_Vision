#include "ConfigManager.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm> // For std::transform

ConfigManager::ConfigManager() : m_configFile("") {
}

bool ConfigManager::loadFromFile(const std::string& configFile) {
    m_configFile = configFile;
    
    std::ifstream file(configFile);
    if (!file.is_open()) {
        std::cerr << "Failed to open configuration file: " << configFile << std::endl;
        return false;
    }
    
    std::stringstream buffer;
    buffer << file.rdbuf();
    
    bool result = parseJson(buffer.str());
    if (!result) {
        std::cerr << "Failed to parse JSON from configuration file: " << configFile << std::endl;
    }
    
    return result;
}

bool ConfigManager::saveToFile(const std::string& configFile) {
    std::string saveFilePath = configFile.empty() ? m_configFile : configFile;
    
    if (saveFilePath.empty()) {
        std::cerr << "No configuration file path specified" << std::endl;
        return false;
    }
    
    std::ofstream file(saveFilePath);
    if (!file.is_open()) {
        std::cerr << "Failed to open file for writing: " << saveFilePath << std::endl;
        return false;
    }
    
    std::string jsonStr = generateJson();
    file << jsonStr;
    
    std::cout << "Configuration saved to: " << saveFilePath << std::endl;
    return true;
}

int ConfigManager::getInt(const std::string& key, int defaultValue) const {
    auto it = m_configValues.find(key);
    if (it == m_configValues.end()) {
        return defaultValue;
    }
    
    try {
        return std::stoi(it->second);
    } catch (...) {
        std::cerr << "Warning: Failed to convert '" << key << "' value to int. Using default: " << defaultValue << std::endl;
        return defaultValue;
    }
}

void ConfigManager::setInt(const std::string& key, int value) {
    m_configValues[key] = std::to_string(value);
}

float ConfigManager::getFloat(const std::string& key, float defaultValue) const {
    auto it = m_configValues.find(key);
    if (it == m_configValues.end()) {
        return defaultValue;
    }
    
    try {
        return std::stof(it->second);
    } catch (...) {
        std::cerr << "Warning: Failed to convert '" << key << "' value to float. Using default: " << defaultValue << std::endl;
        return defaultValue;
    }
}

void ConfigManager::setFloat(const std::string& key, float value) {
    m_configValues[key] = std::to_string(value);
}

std::string ConfigManager::getString(const std::string& key, const std::string& defaultValue) const {
    auto it = m_configValues.find(key);
    if (it == m_configValues.end()) {
        return defaultValue;
    }
    
    return it->second;
}

void ConfigManager::setString(const std::string& key, const std::string& value) {
    m_configValues[key] = value;
}

bool ConfigManager::getBool(const std::string& key, bool defaultValue) const {
    auto it = m_configValues.find(key);
    if (it == m_configValues.end()) {
        return defaultValue;
    }
    
    std::string value = it->second;
    // Convert to lowercase for comparison
    std::transform(value.begin(), value.end(), value.begin(), 
                  [](unsigned char c){ return std::tolower(c); });
    
    if (value == "true" || value == "1" || value == "yes" || value == "y") {
        return true;
    } else if (value == "false" || value == "0" || value == "no" || value == "n") {
        return false;
    } else {
        std::cerr << "Warning: Failed to convert '" << key << "' value to bool. Using default: " << defaultValue << std::endl;
        return defaultValue;
    }
}

void ConfigManager::setBool(const std::string& key, bool value) {
    m_configValues[key] = value ? "true" : "false";
}

void ConfigManager::printValues() const {
    std::cout << "Configuration values:" << std::endl;
    for (const auto& pair : m_configValues) {
        std::cout << "  " << pair.first << " = " << pair.second << std::endl;
    }
}

bool ConfigManager::parseJson(const std::string& jsonStr) {
    // Simple JSON parser for flat key-value structure
    // Not a full JSON parser, but sufficient for our configuration needs
    // Format: { "key1": value1, "key2": value2, ... }
    
    m_configValues.clear();
    
    try {
        // Remove whitespace and newlines
        std::string cleanedJson = jsonStr;
        cleanedJson.erase(std::remove(cleanedJson.begin(), cleanedJson.end(), '\n'), cleanedJson.end());
        cleanedJson.erase(std::remove(cleanedJson.begin(), cleanedJson.end(), '\r'), cleanedJson.end());
        
        // Remove outer braces
        size_t startPos = cleanedJson.find('{');
        size_t endPos = cleanedJson.rfind('}');
        
        if (startPos == std::string::npos || endPos == std::string::npos || startPos >= endPos) {
            std::cerr << "Invalid JSON format: missing braces" << std::endl;
            return false;
        }
        
        std::string content = cleanedJson.substr(startPos + 1, endPos - startPos - 1);
        
        // Parse key-value pairs
        size_t pos = 0;
        while (pos < content.length()) {
            // Find key start (")
            size_t keyStart = content.find('"', pos);
            if (keyStart == std::string::npos) break;
            
            // Find key end (")
            size_t keyEnd = content.find('"', keyStart + 1);
            if (keyEnd == std::string::npos) {
                std::cerr << "Invalid JSON format: unterminated key" << std::endl;
                return false;
            }
            
            // Extract key
            std::string key = content.substr(keyStart + 1, keyEnd - keyStart - 1);
            
            // Find value separator (:)
            size_t valueSep = content.find(':', keyEnd);
            if (valueSep == std::string::npos) {
                std::cerr << "Invalid JSON format: missing value separator for key " << key << std::endl;
                return false;
            }
            
            // Find value end (,) or end of content
            size_t valueEnd = content.find(',', valueSep);
            if (valueEnd == std::string::npos) {
                valueEnd = content.length();
            }
            
            // Extract value (trim whitespace and quotes)
            std::string value = content.substr(valueSep + 1, valueEnd - valueSep - 1);
            value.erase(0, value.find_first_not_of(" \t\n\r\""));
            value.erase(value.find_last_not_of(" \t\n\r\"") + 1);
            
            // Store key-value pair
            m_configValues[key] = value;
            
            // Move to next pair
            pos = valueEnd + 1;
        }
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Exception during JSON parsing: " << e.what() << std::endl;
        return false;
    }
}

std::string ConfigManager::generateJson() const {
    std::stringstream ss;
    ss << "{\n";
    
    size_t count = 0;
    for (const auto& pair : m_configValues) {
        ss << "  \"" << pair.first << "\": ";
        
        // Check if value is a number, boolean, or string
        if (pair.second == "true" || pair.second == "false" || 
            pair.second == "null" || 
            (pair.second.find_first_not_of("-0123456789.") == std::string::npos)) {
            // Number, boolean or null - no quotes
            ss << pair.second;
        } else {
            // String - add quotes
            ss << "\"" << pair.second << "\"";
        }
        
        if (++count < m_configValues.size()) {
            ss << ",";
        }
        
        ss << "\n";
    }
    
    ss << "} \n";
    return ss.str();
} 