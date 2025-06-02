#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <string>
#include <map>

/**
 * @brief Class for managing configuration parameters
 */
class ConfigManager {
public:
    /**
     * @brief Construct a new Config Manager object
     */
    ConfigManager();
    
    /**
     * @brief Load configuration from file
     * 
     * @param configFile Path to the configuration file
     * @return true if loading successful
     * @return false if loading failed
     */
    bool loadFromFile(const std::string& configFile);
    
    /**
     * @brief Save configuration to file
     * 
     * @param configFile Path to save the configuration
     * @return true if saving successful
     * @return false if saving failed
     */
    bool saveToFile(const std::string& configFile = "");
    
    /**
     * @brief Get integer configuration value
     * 
     * @param key Configuration key
     * @param defaultValue Default value if key not found
     * @return int Configuration value
     */
    int getInt(const std::string& key, int defaultValue = 0) const;
    
    /**
     * @brief Set integer configuration value
     * 
     * @param key Configuration key
     * @param value Value to set
     */
    void setInt(const std::string& key, int value);
    
    /**
     * @brief Get float configuration value
     * 
     * @param key Configuration key
     * @param defaultValue Default value if key not found
     * @return float Configuration value
     */
    float getFloat(const std::string& key, float defaultValue = 0.0f) const;
    
    /**
     * @brief Set float configuration value
     * 
     * @param key Configuration key
     * @param value Value to set
     */
    void setFloat(const std::string& key, float value);
    
    /**
     * @brief Get string configuration value
     * 
     * @param key Configuration key
     * @param defaultValue Default value if key not found
     * @return std::string Configuration value
     */
    std::string getString(const std::string& key, const std::string& defaultValue = "") const;
    
    /**
     * @brief Set string configuration value
     * 
     * @param key Configuration key
     * @param value Value to set
     */
    void setString(const std::string& key, const std::string& value);
    
    /**
     * @brief Get boolean configuration value
     * 
     * @param key Configuration key
     * @param defaultValue Default value if key not found
     * @return bool Configuration value
     */
    bool getBool(const std::string& key, bool defaultValue = false) const;
    
    /**
     * @brief Set boolean configuration value
     * 
     * @param key Configuration key
     * @param value Value to set
     */
    void setBool(const std::string& key, bool value);
    
    /**
     * @brief Print all configuration values
     */
    void printValues() const;

private:
    std::string m_configFile;            // Current configuration file path
    std::map<std::string, std::string> m_configValues; // Configuration values storage
    
    /**
     * @brief Parse a JSON string into configuration values
     * 
     * @param jsonStr JSON string to parse
     * @return true if parsing successful
     * @return false if parsing failed
     */
    bool parseJson(const std::string& jsonStr);
    
    /**
     * @brief Generate JSON string from configuration values
     * 
     * @return std::string JSON representation of configuration
     */
    std::string generateJson() const;
};

#endif // CONFIG_MANAGER_H 
 