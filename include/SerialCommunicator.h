#ifndef SERIAL_COMMUNICATOR_H
#define SERIAL_COMMUNICATOR_H

#include <string>
#include <mutex>
#include <queue>
#include <thread>
#include "CommProtocol.h"

/**
 * @brief Command types for control messages
 */
enum class CommandType {
    ERROR,        // Deviation error for line following
    FORWARD,      // Move forward
    STOP,         // Stop movement
    TURN_LEFT,    // Turn left in place
    TURN_RIGHT,   // Turn right in place
    CUSTOM        // Custom command string
};

/**
 * @brief Class for serial communication with control module
 */
class SerialCommunicator {
public:
    /**
     * @brief Construct a new Serial Communicator object
     */
    SerialCommunicator();
    
    /**
     * @brief Destroy the Serial Communicator object
     */
    ~SerialCommunicator();
    
    /**
     * @brief Initialize the serial port
     * 
     * @param portName Name of the serial port (e.g., "COM3" on Windows, "/dev/ttyUSB0" on Linux)
     * @param baudRate Serial baud rate
     * @return true if initialization successful
     * @return false if initialization failed
     */
    bool init(const std::string& portName, int baudRate = 115200);
    
    /**
     * @brief Send deviation data to control module (compatibility method)
     * 
     * @param deviation Deviation value (-1.0 to 1.0)
     * @param lineDetected Whether a valid line was detected
     * @return true if sending successful
     * @return false if sending failed
     */
    bool sendDeviation(float deviation, bool lineDetected);
    
    /**
     * @brief Send a control command (compatibility method)
     * 
     * @param cmd CommandType to send
     * @return true if sending successful
     * @return false if sending failed
     */
    bool sendCommand(CommandType cmd);
    
    /**
     * @brief Send a custom command string (compatibility method)
     * 
     * @param cmd Custom command string
     * @return true if sending successful
     * @return false if sending failed
     */
    bool sendCustomCommand(const std::string& cmd);
    
    /**
     * @brief Send vision data structure 
     * 
     * @param visionData The vision data structure to send
     * @return true if sending was queued successfully
     * @return false if failed to queue sending
     */
    bool sendVisionData(const VisionData& visionData);
    
    /**
     * @brief Check if STM32 data is available to receive (not implemented - always returns false)
     * 
     * @return false always as receiving is disabled
     */
    bool isStm32DataAvailable() { return false; }
    
    /**
     * @brief Receive data from STM32 (not implemented - always fails)
     * 
     * @param stm32Data Reference to store received data
     * @return false always as receiving is disabled
     */
    bool receiveStm32Data(Stm32Data& stm32Data) { return false; }
    
    /**
     * @brief Start communication thread
     * 
     * @return true if successfully started
     * @return false if failed to start
     */
    bool start();
    
    /**
     * @brief Stop communication thread
     */
    void stop();
    
    /**
     * @brief Check if serial port is open and ready
     * 
     * @return true if ready
     * @return false if not ready
     */
    bool isReady() const;

private:
    /**
     * @brief Thread function for processing outgoing messages
     */
    void communicationThread();
    
    /**
     * @brief Convert VisionData to a binary packet for sending
     * 
     * @param visionData The vision data to convert
     * @return std::vector<uint8_t> Binary packet
     */
    std::vector<uint8_t> visionDataToBytes(const VisionData& visionData);
    
    /**
     * @brief Get current timestamp in seconds
     * 
     * @return float Current timestamp
     */
    float getCurrentTimestamp() const;

    /**
     * @brief Convert command type to string (legacy method)
     * 
     * @param cmd Command type to convert
     * @return std::string String representation of the command
     */
    std::string commandToString(CommandType cmd) const;
    
    /**
     * @brief Format deviation data as a string (legacy method)
     * 
     * @param deviation Deviation value
     * @return std::string Formatted string
     */
    std::string formatDeviationMessage(float deviation) const;

    // Platform-specific serial port handle
    #ifdef _WIN32
    void* m_serialHandle; // HANDLE type in Windows
    #else
    int m_serialFd;       // File descriptor in Linux/Unix
    #endif

    std::string m_portName;    // Serial port name
    int m_baudRate;            // Baud rate
    bool m_isRunning;          // Thread running status
    bool m_isReady;            // Port open status
    
    // Thread-safe queue for outgoing messages
    std::mutex m_queueMutex;
    std::queue<VisionData> m_outgoingQueue; // Queue for outgoing vision data packets
    
    std::thread m_commThread;  // Communication thread
};

#endif // SERIAL_COMMUNICATOR_H 