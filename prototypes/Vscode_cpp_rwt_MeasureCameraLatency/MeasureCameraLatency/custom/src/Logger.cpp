
#include <chrono>
#include <cstdio>

#include "../custom/include/Logger.hpp"

#include "vex.h"
#include "../custom/include/robot-config.h"

Logger::Logger()
{
}

Logger::~Logger()
{
}

void Logger::writeLog(std::string data)
{
  testData.push_back(data); // Store the data in the vector for later saving
}

void Logger::dumpLogToTerminal()
{
  for (const std::string &s : testData)
  {
    printf("%s\n", s.c_str());
  }
}

void Logger::textToBrainLcd(std::string msg)
{
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1); // row 1, column 1
  Brain.Screen.print(msg.c_str());
}

void Logger::textToControllerLcd(std::string msg)
{
  controller_1.Screen.clearScreen();
  controller_1.Screen.setCursor(1, 1); // row 1, column 1
  controller_1.Screen.print(msg.c_str());
}

std::string Logger::generateLogFileName()
{
  char logFileName[32];

  // Get current time
  time_t now = time(nullptr);
  struct tm *t = localtime(&now);

  // Format: YYYYMMDD_HHMM.log
  snprintf(logFileName, sizeof(logFileName), "%04d%02d%02d_%02d%02d.log",
           t->tm_year + 1900,
           t->tm_mon + 1,
           t->tm_mday,
           t->tm_hour,
           t->tm_min);

  std::string ret = logFileName;

  return ret;
}

void Logger::resetLog()
{
  // Open the file in write mode
  FILE *logFile = fopen(generateLogFileName().c_str(), "w");

  if (logFile == NULL)
  {
    printf("Failed to open log file for writing.\n");
    return;
  }

  // Write each string in the vector as a new line in the CSV
  for (const std::string &line : testData)
  {
    fprintf(logFile, "%s\n", line.c_str());
  }

  fclose(logFile);

  testData.clear(); // Clear the in-memory log data
}

// Compact timestamp in text form to include in log output
std::string Logger::getTimestamp()
{
  char buf[32];
  snprintf(buf, sizeof(buf), "%u", (unsigned int)Brain.timer(timeUnits::msec));
  return std::string(buf);
}

std::string Logger::to_string(int i)
{
  char buf[32];
  snprintf(buf, sizeof(buf), "%d", i);
  return buf;
}

std::string Logger::to_string(double d)
{
  char buf[32];
  snprintf(buf, sizeof(buf), "%f", d);
  return buf;
}
