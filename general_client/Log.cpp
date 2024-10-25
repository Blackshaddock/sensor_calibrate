#include "Log.h"
#include <sys/stat.h>
#include <unistd.h>

using namespace geosun;

Log& Log::getInstance() {
    static Log instance;
    return instance;
}

Log::Log()
    : currentLogLevel(INFO),
    printToTerminal(false),
    logFilePath("debug.log"),
    baseLogFilePath("debug"),
    maxLogFileSize(10 * 1024 * 1024), // 10MB
    maxBackupFiles(5),
    queueMaxSize(1000),
    lostLogCount(0),
    exitFlag(false),
    performanceMonitoring(false),
    totalLogTime(0),
    maxLogTime(0),
    logCount(0),
    logThread(&Log::processLogQueue, this) {}

Log::~Log() {
    {
        std::lock_guard<std::mutex> lock(queueMutex);
        exitFlag = true;
    }
    queueCV.notify_all();
    if (logThread.joinable()) {
        logThread.join();
    }
}

void Log::setLogLevel(LogLevel level) {
    currentLogLevel = level;
}

void Log::setPrintToTerminal(bool printToTerminal) {
    this->printToTerminal = printToTerminal;
}

void Log::setLogFilePath(const std::string& filePath) {
    logFilePath = filePath;
    baseLogFilePath = logFilePath.substr(0, logFilePath.find_last_of('.'));
}

void Log::setMaxLogFileSize(size_t size) {
    maxLogFileSize = size;
}

void Log::setMaxBackupFiles(int count) {
    maxBackupFiles = count;
}

void Log::setQueueMaxSize(size_t size) {
    queueMaxSize = size;
}

void Log::enablePerformanceMonitoring(bool enable) {
    performanceMonitoring = enable;
}

void Log::logMessage(LogLevel level, const char* file, const char* function, int line, const char* format, ...) {
    if (level < currentLogLevel) {
        return;
    }

    va_list args;
    va_start(args, format);
    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    std::ostringstream oss;
    oss << "[" << getTimestamp() << "] "
        << "[" << getLevelString(level) << "] "
        << "[" << getProcessName() << "-" << file << "-" << function << "() -> " << line << "] "
        << buffer;

    {
        std::lock_guard<std::mutex> lock(queueMutex);
        if (logQueue.size() < queueMaxSize) {
            logQueue.push(oss.str());
            queueCV.notify_one();
        }
        else {
            lostLogCount++;
        }
    }
}

void Log::logHexData(LogLevel level, const void* data, size_t length, const char* file, const char* function, int line) {
    if (level < currentLogLevel) {
        return;
    }

    std::ostringstream oss;
    oss << "[" << getTimestamp() << "] "
        << "[" << getLevelString(level) << "] "
        << "[" << getProcessName() << "-" << file << "-" << function << "() -> " << line << "] "
        << "Hex Data: ";

    const uint8_t* byteData = static_cast<const uint8_t*>(data);
    for (size_t i = 0; i < length; ++i) {
        oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byteData[i]) << " ";
    }

    {
        std::lock_guard<std::mutex> lock(queueMutex);
        if (logQueue.size() < queueMaxSize) {
            logQueue.push(oss.str());
            queueCV.notify_one();
        }
        else {
            lostLogCount++;
        }
    }
}

size_t Log::getCurrentLogFileSize() {
    struct stat st;
    if (stat(logFilePath.c_str(), &st) == 0) {
        return st.st_size;
    }
    return 0;
}

std::string Log::getBackupLogFileName(int index) {
    return baseLogFilePath + "_" + std::to_string(index) + ".log";
}

void Log::rotateLogFiles() {
    
    if (std::ifstream(getBackupLogFileName(maxBackupFiles)).good()) {
        std::remove(getBackupLogFileName(maxBackupFiles).c_str());
    }

    
    for (int i = maxBackupFiles - 1; i > 0; --i) {
        std::string oldFile = getBackupLogFileName(i);
        std::string newFile = getBackupLogFileName(i + 1);
        if (std::ifstream(oldFile).good()) {
            std::rename(oldFile.c_str(), newFile.c_str());
        }
    }

    
    std::rename(logFilePath.c_str(), getBackupLogFileName(1).c_str());

    
    std::ofstream newLogFile(logFilePath);
    if (newLogFile.is_open()) {
        newLogFile.close();
    }
}

void Log::processLogQueue() {
    while (true) {
        std::unique_lock<std::mutex> lock(queueMutex);
        queueCV.wait(lock, [this] { return !logQueue.empty() || exitFlag; });

        while (!logQueue.empty()) {
            std::string logEntry = logQueue.front();
            logQueue.pop();

            lock.unlock();

            auto start = std::chrono::high_resolution_clock::now();

            if (printToTerminal) {
                std::cout << logEntry << std::endl;
            }

            if (getCurrentLogFileSize() >= maxLogFileSize) {
                rotateLogFiles();
            }

            std::ofstream logFile(logFilePath,  std::ios_base::app);
            if (logFile.is_open()) {
                logFile << logEntry << std::endl;
            }

            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> duration = end - start;

            if (performanceMonitoring) {
                std::lock_guard<std::mutex> perfLock(performanceMutex);
                totalLogTime += duration.count();
                if (duration.count() > maxLogTime) {
                    maxLogTime = duration.count();
                }
                logCount++;
            }

            lock.lock();
        }

        if (exitFlag && logQueue.empty()) {
            break;
        }
    }
}

std::string Log::getTimestamp() {
    std::time_t now = std::time(nullptr);
    char timeStr[100];
    std::strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
    return std::string(timeStr);
}

std::string Log::getLevelString(LogLevel level) {
    switch (level) {
    case DEBUG: return "DEBUG";
    case INFO: return "INFO";
    case WARNING: return "WARNING";
    case ERROR: return "ERROR";
    default: return "UNKNOWN";
    }
}

std::string Log::getProcessName() {
    char path[256];
    char processName[256] = { 0 };
    sprintf(path, "/proc/%d/cmdline", getpid());
    FILE* cmdline = fopen(path, "r");
    if (cmdline) {
        fread(processName, sizeof(processName), 1, cmdline);
        fclose(cmdline);
    }
    return std::string(processName);
}

size_t Log::getLostLogCount() const {
    return lostLogCount;
}
