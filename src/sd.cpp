#include "sd.hpp"

boolean flush_flag = false;
uint8_t LogSize = 10;

QueueHandle_t sdQueue;

File logFile;

void SD_setup(uint8_t logsize) {
    LogSize = logsize;
    typedef LogData LogdataArray[LogSize];

    sdQueue = xQueueCreate(10, sizeof(LogdataArray));

    // Core0にSDタスクを作成
    xTaskCreatePinnedToCore(
        sdTask,           // タスク関数
        "sdTask",         // タスク名
        4096,             // スタックサイズ
        NULL,             // 引数
        1,                // 優先度
        NULL,             // タスクハンドル
        0                 // Core0を指定
    );
}

String makeUniqueFilename(const char* basePath) {
  String path = basePath;

  // 拡張子の位置を探す
  int dotIndex = path.lastIndexOf('.');
  String name = (dotIndex >= 0) ? path.substring(0, dotIndex) : path;
  String ext  = (dotIndex >= 0) ? path.substring(dotIndex) : "";

  if (!SD.exists(path)) {
    return path;  // そのまま使える
  }

  for (int i = 1; i < 10000; i++) {
    String newPath = name + "_" + String(i) + ext;
    if (!SD.exists(newPath)) {
      return newPath;
    }
  }

  return ""; // 異常（全部埋まってる）
}

void sdTask(void *arg){
    LogData LogBuffer[LogSize];

    char dataString[1024];
    size_t idx = 0;

    for (;;) {
        if(uxQueueMessagesWaiting(sdQueue) > 0){
            xQueueReceive(sdQueue, LogBuffer, 0);

            for(int i = 0; i < LogSize; i++){
                idx += snprintf(dataString + idx, sizeof(dataString) - idx, "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                    LogBuffer[i].time_ms,
                    LogBuffer[i].m1_deg,LogBuffer[i].m1_cur,LogBuffer[i].m1_torq,
                    LogBuffer[i].m2_deg,LogBuffer[i].m2_cur,LogBuffer[i].m2_torq,
                    LogBuffer[i].m3_deg,LogBuffer[i].m3_cur,LogBuffer[i].m3_torq,
                    LogBuffer[i].m4_deg,LogBuffer[i].m4_cur,LogBuffer[i].m4_torq,
                    LogBuffer[i].front,LogBuffer[i].back);
            }

            logFile.print(dataString);
            logFile.flush();
            idx = 0;
        }
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}