#ifndef SD_HPP
#define SD_HPP

#include <Arduino.h>
#include <SD.h>

extern boolean flush_flag;

typedef struct {
    unsigned long time_ms;
    float m1_deg;
    float m1_cur;
    float m1_torq;
    float m2_deg;
    float m2_cur;
    float m2_torq;
    float m3_deg;
    float m3_cur;
    float m3_torq;
    float m4_deg;
    float m4_cur;
    float m4_torq;
    float front;
    float back;
} LogData;
extern uint8_t LogSize;

extern QueueHandle_t sdQueue;

extern File logFile;

void SD_setup(uint8_t logsize);
String makeUniqueFilename(const char* basePath);
void sdTask(void *arg);

#endif