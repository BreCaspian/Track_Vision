#include "SerialCommunicator.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <thread>
#include <chrono>
#include <vector>
#include <cstring>

// Platform-specific includes
#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

SerialCommunicator::SerialCommunicator()
    : m_baudRate(115200),
      m_isRunning(false),
      m_isReady(false) {
#ifdef _WIN32
    m_serialHandle = INVALID_HANDLE_VALUE;
#else
    m_serialFd = -1;
#endif
    // 不再需要初始化接收缓冲区
}

SerialCommunicator::~SerialCommunicator() {
    stop();
    
    // Close serial port if still open
#ifdef _WIN32
    if (m_serialHandle != INVALID_HANDLE_VALUE) {
        CloseHandle(m_serialHandle);
        m_serialHandle = INVALID_HANDLE_VALUE;
    }
#else
    if (m_serialFd >= 0) {
        close(m_serialFd);
        m_serialFd = -1;
    }
#endif

    m_isReady = false;
}

bool SerialCommunicator::init(const std::string& portName, int baudRate) {
    // Stop any running communication
    stop();
    
    m_portName = portName;
    m_baudRate = baudRate;
    
#ifdef _WIN32
    // Windows implementation
    std::wstring widePortName = L"\\\\.\\" + std::wstring(portName.begin(), portName.end());
    
    m_serialHandle = CreateFileW(
        widePortName.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0,                  // No sharing
        NULL,               // Default security
        OPEN_EXISTING,      // Open existing port
        0,                  // Non-overlapped I/O
        NULL                // No template
    );
    
    if (m_serialHandle == INVALID_HANDLE_VALUE) {
        std::cerr << "Error: Could not open serial port " << portName << std::endl;
        return false;
    }
    
    // Configure serial port
    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    
    if (!GetCommState(m_serialHandle, &dcbSerialParams)) {
        std::cerr << "Error: Could not get serial port state" << std::endl;
        CloseHandle(m_serialHandle);
        m_serialHandle = INVALID_HANDLE_VALUE;
        return false;
    }
    
    dcbSerialParams.BaudRate = baudRate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    
    if (!SetCommState(m_serialHandle, &dcbSerialParams)) {
        std::cerr << "Error: Could not set serial port state" << std::endl;
        CloseHandle(m_serialHandle);
        m_serialHandle = INVALID_HANDLE_VALUE;
        return false;
    }
    
    // Set timeouts
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 10;
    
    if (!SetCommTimeouts(m_serialHandle, &timeouts)) {
        std::cerr << "Error: Could not set serial port timeouts" << std::endl;
        CloseHandle(m_serialHandle);
        m_serialHandle = INVALID_HANDLE_VALUE;
        return false;
    }
#else
    // Linux/Unix implementation
    m_serialFd = open(portName.c_str(), O_RDWR | O_NOCTTY);
    
    if (m_serialFd < 0) {
        std::cerr << "Error: Could not open serial port " << portName << std::endl;
        return false;
    }
    
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    
    if (tcgetattr(m_serialFd, &tty) != 0) {
        std::cerr << "Error: Could not get serial port attributes" << std::endl;
        close(m_serialFd);
        m_serialFd = -1;
        return false;
    }
    
    // Set baud rate
    speed_t baud;
    switch (baudRate) {
        case 9600:   baud = B9600;   break;
        case 19200:  baud = B19200;  break;
        case 38400:  baud = B38400;  break;
        case 57600:  baud = B57600;  break;
        case 115200: baud = B115200; break;
        default:
            std::cerr << "Error: Unsupported baud rate: " << baudRate << std::endl;
            close(m_serialFd);
            m_serialFd = -1;
            return false;
    }
    
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);
    
    // 8N1 (8 bits, no parity, 1 stop bit)
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    
    // No hardware flow control
    tty.c_cflag &= ~CRTSCTS;
    
    // Enable receiver and ignore modem control lines
    tty.c_cflag |= CREAD | CLOCAL;
    
    // Raw input
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    // Raw output
    tty.c_oflag &= ~OPOST;
    
    // No software flow control
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    
    // No special handling of bytes
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    
    // Set read timeouts
    tty.c_cc[VMIN] = 0;  // Non-blocking read
    tty.c_cc[VTIME] = 1; // 100ms timeout
    
    if (tcsetattr(m_serialFd, TCSANOW, &tty) != 0) {
        std::cerr << "Error: Could not set serial port attributes" << std::endl;
        close(m_serialFd);
        m_serialFd = -1;
        return false;
    }
#endif
    
    m_isReady = true;
    std::cout << "Serial port " << portName << " initialized at " << baudRate << " baud" << std::endl;
    
    return true;
}

bool SerialCommunicator::sendDeviation(float deviation, bool lineDetected) {
    if (!m_isReady) {
        std::cerr << "Error: Serial port not ready" << std::endl;
        return false;
    }
    
    // 使用结构体通信代替文本
    VisionData data;
    data.line_offset = deviation;
    data.line_found = lineDetected ? 1 : 0;
    data.obstacle_detected = 0; // 默认没有检测到障碍物
    data.avoid_direction = AVOID_NONE;
    data.timestamp = getCurrentTimestamp();
    
    return sendVisionData(data);
}

bool SerialCommunicator::sendCommand(CommandType cmd) {
    if (!m_isReady) {
        std::cerr << "Error: Serial port not ready" << std::endl;
        return false;
    }
    
    // 使用结构体通信代替文本
    VisionData data;
    data.timestamp = getCurrentTimestamp();
    
    switch (cmd) {
        case CommandType::STOP:
            data.obstacle_detected = 1;
            data.avoid_direction = AVOID_NONE;
            break;
        case CommandType::TURN_LEFT:
            data.obstacle_detected = 1;
            data.avoid_direction = AVOID_LEFT;
            break;
        case CommandType::TURN_RIGHT:
            data.obstacle_detected = 1;
            data.avoid_direction = AVOID_RIGHT;
            break;
        case CommandType::FORWARD:
            data.obstacle_detected = 0;
            data.avoid_direction = AVOID_NONE;
            break;
        default:
            break;
    }
    
    return sendVisionData(data);
}

bool SerialCommunicator::sendCustomCommand(const std::string& cmd) {
    // 保留此方法仅用于兼容性，实际上不再需要
    std::cerr << "Warning: sendCustomCommand is deprecated with binary protocol" << std::endl;
    return false;
}

bool SerialCommunicator::sendVisionData(const VisionData& visionData) {
    if (!m_isReady) {
        std::cerr << "Error: Serial port not ready" << std::endl;
        return false;
    }
    
    // 添加到发送队列
    std::lock_guard<std::mutex> lock(m_queueMutex);
    m_outgoingQueue.push(visionData);
    
    return true;
}

bool SerialCommunicator::start() {
    if (m_isRunning) {
        return true; // Already running
    }
    
    if (!m_isReady) {
        std::cerr << "Error: Serial port not ready" << std::endl;
        return false;
    }
    
    m_isRunning = true;
    m_commThread = std::thread(&SerialCommunicator::communicationThread, this);
    
    std::cout << "Serial communication thread started" << std::endl;
    return true;
}

void SerialCommunicator::stop() {
    m_isRunning = false;
    
    if (m_commThread.joinable()) {
        m_commThread.join();
    }
}

bool SerialCommunicator::isReady() const {
    return m_isReady;
}

void SerialCommunicator::communicationThread() {
    while (m_isRunning) {
        // 处理发送队列
        {
            std::lock_guard<std::mutex> lock(m_queueMutex);
            while (!m_outgoingQueue.empty()) {
                VisionData data = m_outgoingQueue.front();
                m_outgoingQueue.pop();
                
                // 直接发送VisionData结构体
#ifdef _WIN32
                DWORD bytesWritten = 0;
                if (!WriteFile(m_serialHandle, &data, sizeof(VisionData), &bytesWritten, NULL)) {
                    std::cerr << "Error: Failed to write to serial port" << std::endl;
                }
#else
                ssize_t bytesWritten = write(m_serialFd, &data, sizeof(VisionData));
                if (bytesWritten != sizeof(VisionData)) {
                    std::cerr << "Error: Failed to write to serial port" << std::endl;
                }
#endif
            }
        }
        
        // 降低CPU使用率
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

std::vector<uint8_t> SerialCommunicator::visionDataToBytes(const VisionData& visionData) {
    std::vector<uint8_t> bytes(sizeof(VisionData));
    memcpy(bytes.data(), &visionData, sizeof(VisionData));
    return bytes;
}

float SerialCommunicator::getCurrentTimestamp() const {
    using namespace std::chrono;
    auto now = steady_clock::now();
    return duration<float>(now.time_since_epoch()).count();
}

// 以下方法保留兼容性，但实际不再使用
std::string SerialCommunicator::commandToString(CommandType cmd) const {
    switch (cmd) {
        case CommandType::FORWARD:   return "CMD:FORWARD\n";
        case CommandType::STOP:      return "CMD:STOP\n";
        case CommandType::TURN_LEFT: return "CMD:TURN_LEFT\n";
        case CommandType::TURN_RIGHT:return "CMD:TURN_RIGHT\n";
        default:                     return "";
    }
}

std::string SerialCommunicator::formatDeviationMessage(float deviation) const {
    // 将偏差转换为-32767到32767范围的整数
    int16_t deviationInt = static_cast<int16_t>(deviation * 32767.0f);
    
    std::stringstream ss;
    ss << "ERR:" << deviationInt << "\n";
    return ss.str();
} 