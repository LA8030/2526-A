#pragma once

#include <string>
#include <vector>
#include <cstdio>

class Logger
{
public:
    Logger();
    ~Logger();

    void writeLog(std::string data);
    void dumpLogToTerminal();
    void textToControllerLcd(std::string msg);
    void resetLog();
    std::string getTimestamp();   
    std::string to_string(double d);

private:
    std::string generateLogFileName();

    FILE *logFile = nullptr;           // Global file pointer for logging
    std::vector<std::string> testData; // Vector to store test data for later saving
};