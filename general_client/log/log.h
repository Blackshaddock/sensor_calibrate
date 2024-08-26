/*
 * @Date: 2024-08-05 14:20:52
 * @LastEditors: aodp aodp@qq.com
 * @LastEditTime: 2024-08-13 17:24:22
 * @FilePath: /mylog/Log.h
 */
/**
 * @file Log.h
 * @author  aodp (aodp@qq.com)
 * @brief 
 * @version 0.1
 * @date 2024-08-09
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef LOG_H
#define LOG_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <queue>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <chrono>
#include <iomanip>
#include <ctime>
#include <cstdarg>

enum LogLevel {
    DEBUG,
    INFO,
    WARNING,
    ERROR
};

class Log {
public:
    static Log& getInstance();
    void setLogLevel(LogLevel level);
    void setPrintToTerminal(bool printToTerminal);
    void setLogFilePath(const std::string& filePath);
    void setMaxLogFileSize(size_t size);
    void setMaxBackupFiles(int count);
    void setQueueMaxSize(size_t size);
    void enablePerformanceMonitoring(bool enable);
    void logMessage(LogLevel level, const char* file, const char* function, int line, const char* format, ...);
    void logHexData(LogLevel level, const void* data, size_t length, const char* file, const char* function, int line);
    size_t getLostLogCount() const;

    // 添加的访问方法
    double getTotalLogTime() const {
        return totalLogTime;
    }

    size_t getLogCount() const {
        return logCount;
    }

    double getMaxLogTime() const {
        return maxLogTime;
    }

private:
    Log();
    ~Log();
    Log(const Log&) = delete;
    Log& operator=(const Log&) = delete;

    void processLogQueue();
    void rotateLogFiles();
    size_t getCurrentLogFileSize();
    std::string getBackupLogFileName(int index);
    std::string getTimestamp();
    std::string getLevelString(LogLevel level);
    std::string getProcessName();

    LogLevel currentLogLevel;
    bool printToTerminal;
    std::string logFilePath;
    std::string baseLogFilePath;
    size_t maxLogFileSize;
    int maxBackupFiles;
    size_t queueMaxSize;
    size_t lostLogCount;
    bool exitFlag;
    bool performanceMonitoring;
    double totalLogTime;
    double maxLogTime;
    size_t logCount;
    std::queue<std::string> logQueue;
    std::mutex queueMutex;
    std::mutex performanceMutex;
    std::condition_variable queueCV;
    std::thread logThread;
};

#define LOG_DEBUG(format, ...) Log::getInstance().logMessage(DEBUG, __FILE__, __FUNCTION__, __LINE__, format, ##__VA_ARGS__)
#define LOG_INFO(format, ...) Log::getInstance().logMessage(INFO, __FILE__, __FUNCTION__, __LINE__, format, ##__VA_ARGS__)
#define LOG_WARNING(format, ...) Log::getInstance().logMessage(WARNING, __FILE__, __FUNCTION__, __LINE__, format, ##__VA_ARGS__)
#define LOG_ERROR(format, ...) Log::getInstance().logMessage(ERROR, __FILE__, __FUNCTION__, __LINE__, format, ##__VA_ARGS__)
#define LOG_HEX(level, data, length) Log::getInstance().logHexData(level, data, length, __FILE__, __FUNCTION__, __LINE__)

#endif // LOG_H
