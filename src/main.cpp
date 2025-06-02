#include <iostream>
#include <string>
#include "track_vision/core/TrackVision.h"

void printUsage(const char* programName) {
    std::cout << "Usage: " << programName << " [-c config_file]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  -c config_file    Path to configuration file (default: config/vision_config.json)" << std::endl;
}

int main(int argc, char** argv) {
    std::string configFile = "config/vision_config.json";
    
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-c" && i + 1 < argc) {
            configFile = argv[++i];
        } else if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            return 0;
        } else {
            std::cerr << "Unknown option: " << arg << std::endl;
            printUsage(argv[0]);
            return 1;
        }
    }
    
    std::cout << "Track Vision System" << std::endl;
    std::cout << "Using config file: " << configFile << std::endl;
    
    // Create and initialize TrackVision
    track_vision::core::TrackVision vision(configFile);
    
    if (!vision.init()) {
        std::cerr << "Failed to initialize TrackVision system" << std::endl;
        return 1;
    }
    
    std::cout << "System initialized. Starting main loop..." << std::endl;
    vision.run();
    
    std::cout << "System stopped." << std::endl;
    return 0;
} 