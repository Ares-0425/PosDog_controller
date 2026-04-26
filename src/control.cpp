#include "control.hpp"
#include <cmath>
#include "display.hpp"
#include "can.hpp"
#include "imu.hpp"
#include "sd.hpp"

float received_value = 0.0; // 受信した値を格納する変数
float control_value = 0.0; // コントロール値を格納する変数

char serial_buffer[64]; // シリアルからの入力を格納するバッファ
String serial_input;
float serial_value = 0.0;

volatile uint8_t Loop_flag = 0;

// 割り込み関数
// Intrupt function
hw_timer_t* timer = NULL;
void IRAM_ATTR onTimer() {
    Loop_flag = 1;
}

void canTest(void) {
    int receive_id = 0x201;
    int send_id = 0x101;

    display_clear(); // ディスプレイをクリア
    display_println("CAN Test Started");

    tx_message.identifier = send_id;
    tx_message.data_length_code = 1;
    tx_message.data[0] = 'R';
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK) {
        //Serial.println("CAN Message Sent!");
    } else {
        //Serial.println("CAN Send Failed");
    }
    Serial.println("Motor Restart");
    int count = 0;
    boolean cantest_flag = false;
    do{
        delay(1);
        count++;
        if (count > 10000) { // タイムアウト処理
            display_println("Timeout waiting for 'R'");
            delay(1000);
            cantest_flag = true;
        }
        if(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
            if(rx_message.identifier == receive_id && rx_message.data[0] == 'R'){
                cantest_flag = true;
            }
        }
    }while(cantest_flag == false);

    tx_message.data[0] = 'A';
    tx_message.data_length_code = 1;
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
    count = 0;
    cantest_flag = false;
    do{
        delay(1);
        count++;
        if (count > 3000) { // タイムアウト処理
            display_println("Timeout waiting for 'A'");
            cantest_flag = true;
        }
        if(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
            if(rx_message.identifier == receive_id && rx_message.data[0] == 'A'){
                cantest_flag = true;
            }
        }
    }while(cantest_flag == false);
    received_value = *((float*)&rx_message.data[1]);
    M5.Display.startWrite();
    M5.Display.print("Shaft Angle:");
    M5.Display.println(received_value);
    M5.Display.endWrite();

    tx_message.data[0] = 'V';
    tx_message.data_length_code = 1;
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
    count = 0;
    cantest_flag = false;
    do{
        delay(1);
        count++;
        if (count > 3000) { // タイムアウト処理
            display_println("Timeout waiting for 'V'");
            cantest_flag = true;
        }
        if(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
            if(rx_message.identifier == receive_id && rx_message.data[0] == 'V'){
                cantest_flag = true;
            }
        }
    }while(cantest_flag == false);
    received_value = *((float*)&rx_message.data[1]);
    M5.Display.startWrite();
    M5.Display.print("Shaft Velocity:");
    M5.Display.println(received_value);
    M5.Display.endWrite();
    delay(1000);

    tx_message.data[0] = 'M';
    tx_message.data[1] = 'T';
    ((float*)&tx_message.data[2])[0] = 0.0;
    tx_message.data_length_code = 2 + sizeof(float); // 6
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
    display_println("Torque Mode Set 0.0");

    sendStart();
    display_println("Motor Started");
    delay(3000);

    tx_message.data[0] = 'T';
    ((float*)&tx_message.data[1])[0] = 0.1;
    tx_message.data_length_code = 1 + sizeof(float); // 5
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
    display_println("Torque Set 0.1");
    delay(500);

    tx_message.data[0] = 'T';
    ((float*)&tx_message.data[1])[0] = 0.0;
    tx_message.data_length_code = 1 + sizeof(float); // 5
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
    display_println("Torque Set 0.0");
    delay(5000);

    tx_message.data[0] = 'M';
    tx_message.data[1] = 'S';
    ((float*)&tx_message.data[2])[0] = 0.0;
    tx_message.data_length_code = 2 + sizeof(float); // 6
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
    display_println("Speed Mode Set 0.0");
    delay(1000);

    tx_message.data[0] = 'S';
    ((float*)&tx_message.data[1])[0] = -30.0; // 速度を0.1に設定
    tx_message.data_length_code = 1 + sizeof(float); // 5
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
    display_println("Speed Set -30");
    delay(1000);

    tx_message.data[0] = 'S';
    ((float*)&tx_message.data[1])[0] = 0.0; // 速度を0.0に設定
    tx_message.data_length_code = 1 + sizeof(float); // 5
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
    display_println("Speed Set 0.0");
    delay(1000);

    tx_message.data[0] = 'A';
    tx_message.data_length_code = 1;
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
    count = 0;
    cantest_flag = false;
    do{
        delay(1);
        count++;
        if (count > 3000) { // タイムアウト処理
            display_println("Timeout waiting for 'A'");
            cantest_flag = true;
        }
        if(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
            if(rx_message.identifier == receive_id && rx_message.data[0] == 'A'){
                cantest_flag = true;
            }
        }
    }while(cantest_flag == false);
    received_value = *((float*)&rx_message.data[1]);

    tx_message.data[0] = 'M';
    tx_message.data[1] = 'P';
    ((float*)&tx_message.data[2])[0] = received_value;
    tx_message.data_length_code = 2 + sizeof(float); // 6
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
    display_println("Position Mode Set 0.0");
    delay(1000);

    tx_message.data[0] = 'P';
    ((float*)&tx_message.data[1])[0] = received_value + 1.0; // 位置を1に設定
    tx_message.data_length_code = 1 + sizeof(float); // 5
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
    display_println("Position Set 1.0");
    delay(1000);

    tx_message.data[0] = 'P';
    ((float*)&tx_message.data[1])[0] = received_value; // 位置を0に設定
    tx_message.data_length_code = 1 + sizeof(float); // 5
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
    display_println("Position Set 0.0");
    delay(1000);

    sendStop();
    display_println("Motor Stopped");
    display_println("CAN Test Completed");
    delay(1000);

    display_clear();
    display_menu();
}

void sineWave(){
    const int control_cycle = 5000; //制御周期(us)
    const int gear_rate = 16;
    const float max_degree = PI/4.0f;    //45°
    float sine_fleq = 0.5; //sine波周波数(Hz)

    float degree_1,degree_2,degree_3,degree_4;
    float time = 0.0;

    boolean sinewave_flag = false;

    display_clear();
    display_println("sine_wave");

    // ドライバーリセット
    for(int i = 1; i <= 4; i++){
        tx_message.identifier = 0x100 + i;
        tx_message.data[0] = 'R';
        tx_message.data_length_code = 1;
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
        int count = 0;
        sinewave_flag = false;
        do{
            delay(1);
            count++;
            if (count > 10000) { // タイムアウト処理
                Serial.print("timeout");
                sinewave_flag = true;
            }
            if(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
                if(rx_message.identifier == 0x200 + i && rx_message.data[0] == 'R'){
                    sinewave_flag = true;
                }
            }
        }while(sinewave_flag == false);
        Serial.printf("%#x:reset\n",0x100+i);
    }

    // キャリブレーション
    for(int i = 1; i <= 4; i++){
        tx_message.identifier = 0x100 + i;
        tx_message.data[0] = 'L';
        tx_message.data_length_code = 1;
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
        int count = 0;
        sinewave_flag = false;
        do{
            delay(1);
            count++;
            if (count > 5000) { // タイムアウト処理
                Serial.print("timeout");
                sinewave_flag = true;
            }
            if(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
                if(rx_message.identifier == 0x200 + i && rx_message.data[0] == 'L'){
                    sinewave_flag = true;
                }
            }
        }while(sinewave_flag == false);
        Serial.printf("%#x:calib\n",0x100+i);
    }

    // 割り込み設定
    // Initialize intrupt
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, control_cycle, true);
    timerAlarmEnable(timer);

    while(true){
        // 割り込みにより200Hzで以降のコードが実行
        while (Loop_flag == 0);
        Loop_flag = 0;
        M5.update();
        if(M5.BtnA.isPressed()){
            break;
        }

        unsigned long t = millis();

        time += control_cycle/1000;
        degree_1 = max_degree * sin(2*PI*sine_fleq*time/1000 + 0);
        degree_2 = -max_degree * sin(2*PI*sine_fleq*time/1000 + PI);
        degree_3 = max_degree * sin(2*PI*sine_fleq*time/1000 + PI/4);
        degree_4 = -max_degree * sin(2*PI*sine_fleq*time/1000 + 3*PI/4);

        tx_message.identifier = 0x101;
        tx_message.data[0] = 'P';
        ((float*)&tx_message.data[1])[0] = degree_1;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
        Serial.printf("%f ",*((float*)&tx_message.data[1]));

        tx_message.identifier = 0x102;
        tx_message.data[0] = 'P';
        ((float*)&tx_message.data[1])[0] = degree_2;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
        Serial.printf("%f ",*((float*)&tx_message.data[1]));

        tx_message.identifier = 0x103;
        tx_message.data[0] = 'P';
        ((float*)&tx_message.data[1])[0] = degree_3;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
        Serial.printf("%f ",*((float*)&tx_message.data[1]));

        tx_message.identifier = 0x104;
        tx_message.data[0] = 'P';
        ((float*)&tx_message.data[1])[0] = degree_4;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
        Serial.printf("%f\n",*((float*)&tx_message.data[1]));
    }
    timerAlarmDisable(timer);
    display_clear();
    display_menu();
}

void imuMode(void){
    const int control_cycle = 5000; //制御周期(us)
    const float front_deg = 20;
    const float front_deg_range = 3;
    float time = 0.0;
    float roll, pitch, yaw;
    float gyroX, gyroY, gyroZ;
    float omega;
    float phi_hat = 0.0f;          // 推定位相
    float omega_hat = 0.0f;        // 推定周波数

    display_clear();
    display_println("imu");

    const int BUFFER_SIZE = 10;
    int bufferCount = 0;     // バッファ内データ件数

    LogData logData[BUFFER_SIZE] = {0};

    if (!SD.begin(4)) {
        Serial.println("SDカードの初期化に失敗しました");
        while (1);
    }

    String filename = makeUniqueFilename("/gyro_log.csv");
    logFile = SD.open(filename, FILE_WRITE);
    if (!logFile) {
        Serial.println("ファイルオープン失敗");
        while (1);
    }

    // ヘッダー
    logFile.println("time_ms,roll,pitch,yaw,omega");
    logFile.flush();
    
    int count = 0;
    while(count < 50){    //一定角度で安定するまで待機
        kalman_update(&roll,&pitch,&yaw);
        if(pitch <= front_deg + front_deg_range && pitch >= front_deg - front_deg_range){
            count++;
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("imu");
            canvas.setTextSize(3);
            canvas.setTextColor(GREEN,BLACK);
            canvas.drawCenterString(String(pitch,3), M5.Display.width() / 2, M5.Display.height() / 2);
            canvas.pushSprite(0, 0);
        }else{
            count = 0;
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("imu");
            canvas.setTextSize(3);
            canvas.drawCenterString(String(pitch,3), M5.Display.width() / 2, M5.Display.height() / 2);
            canvas.pushSprite(0, 0);
        }
    }
    Serial.println("degree ready");
    // */
    while(pitch >= 0){    //0°を跨ぐまで待機
        kalman_update(&roll,&pitch,&yaw);
        if(pitch <= front_deg + front_deg_range && pitch >= front_deg - front_deg_range){
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("imu");
            canvas.setTextSize(3);
            canvas.setCursor(0,20);
            canvas.print("degree ready");
            canvas.setTextColor(GREEN,BLACK);
            canvas.drawCenterString(String(pitch,3), M5.Display.width() / 2, M5.Display.height() / 2);
            canvas.pushSprite(0, 0);
        }else{
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("imu");
            canvas.setTextSize(3);
            canvas.setCursor(0,20);
            canvas.print("degree ready");
            canvas.drawCenterString(String(pitch,3), M5.Display.width() / 2, M5.Display.height() / 2);
            canvas.pushSprite(0, 0);
        }
    }
    Serial.println("GO");

    M5.update();
    M5.Imu.getGyroData(&gyroX, &gyroY, &gyroZ);
    if(gyroY > 0){
        while(gyroY >= 0){
            M5.update();
            kalman_update(&roll,&pitch,&yaw);
            M5.Imu.getGyroData(&gyroX, &gyroY, &gyroZ);
            //sine_fleq = estimatePeriod(pitch, gyroY);
        }
    }else{
        while(gyroY <= 0){
            M5.update();
            kalman_update(&roll,&pitch,&yaw);
            M5.Imu.getGyroData(&gyroX, &gyroY, &gyroZ);
            //sine_fleq = estimatePeriod(pitch, gyroY);
        }
    }
    
    count = 0;

    SD_setup(BUFFER_SIZE);

    // 割り込み設定
    // Initialize intrupt
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, control_cycle, true);
    timerAlarmEnable(timer);

    while(true){
        // 割り込みにより200Hzで以降のコードが実行
        while (Loop_flag == 0);
        Loop_flag = 0;
        M5.update();
        if(M5.BtnA.isPressed()){
            break;
        }

        kalman_update(&roll,&pitch,&yaw);
        M5.Imu.getGyroData(&gyroX, &gyroY, &gyroZ);
        omega = PLL(pitch, &omega_hat, &phi_hat, control_cycle / 1000);

        logData[bufferCount].time_ms = millis();
        logData[bufferCount].m1_deg = roll;
        logData[bufferCount].m1_cur = pitch;
        logData[bufferCount].m2_deg = yaw;
        logData[bufferCount].m2_cur = omega;
        
        bufferCount++;

        if (bufferCount >= BUFFER_SIZE) {
            xQueueSend(sdQueue, logData, portMAX_DELAY);
            bufferCount = 0;
        }
    }
    timerAlarmDisable(timer);
    xQueueSend(sdQueue, logData, portMAX_DELAY);

    delay(1000);
    logFile.close();

    display_clear();
    display_menu();
}

void imuMode2(void){
    const int control_cycle = 5000; //制御周期(us)
    const float front_deg = 20;
    const float front_deg_range = 3;
    float time = 0.0;
    float roll1, pitch1, yaw1;
    float roll2, pitch2, yaw2;
    float omega1, omega2;
    float phi_hat1 = 0.0f;          // 推定位相
    float phi_hat2 = 0.0f;
    float omega_hat1 = 0.0f;        // 推定周波数
    float omega_hat2 = 0.0f;
    sensors_event_t a1, m1, g1, temp1;
    sensors_event_t a2, m2, g2, temp2; 
    SimpleKalmanFilter kalman_Pitch1;
    SimpleKalmanFilter kalman_Pitch2;
    SimpleKalmanFilter kalman_Roll1;
    SimpleKalmanFilter kalman_Roll2;
    SimpleKalmanFilter kalman_Yaw1;
    SimpleKalmanFilter kalman_Yaw2;
    float pitchAcc1, pitchAcc2, rollAcc1, rollAcc2;
    int now_time, dt, last_time = 0;

    display_clear();
    display_println("imu");

    const int BUFFER_SIZE = 10;
    int bufferCount = 0;     // バッファ内データ件数

    LogData logData[BUFFER_SIZE] = {0};

    if (!SD.begin(4)) {
        Serial.println("SDカードの初期化に失敗しました");
        display_println("SDカードの初期化に失敗しました");
        while (M5.BtnB.isPressed() == false) M5.update();
    }

    String filename = makeUniqueFilename("/gyro_log.csv");
    logFile = SD.open(filename, FILE_WRITE);
    if (!logFile) {
        Serial.println("ファイルオープン失敗");
        display_println("ファイルオープン失敗");
        while (M5.BtnB.isPressed() == false) M5.update();
    }

    // ヘッダー
    logFile.println("time_ms,roll_1,pitch_1,yaw_1,omega_1,roll_2,pitch_2,yaw_2,omega_2");
    logFile.flush();
    
    int count = 0;

    SD_setup(BUFFER_SIZE);

    // 割り込み設定
    // Initialize intrupt
    timer = timerBegin(0, getApbFrequency()/1000000, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, control_cycle, true);
    timerAlarmEnable(timer);

    while(true){
        // 割り込みにより200Hzで以降のコードが実行
        while (Loop_flag == 0);
        Loop_flag = 0;
        M5.update();
        if(M5.BtnA.isPressed()){
            break;
        }
        now_time = millis();
        dt = now_time - last_time;
        last_time = now_time;

        lsm1.getEvent(&a1, &m1, &g1, &temp1);
        lsm2.getEvent(&a2, &m2, &g2, &temp2);
        rollAcc1 = atan2(-a1.acceleration.y, sqrt(a1.acceleration.x * a1.acceleration.x + a1.acceleration.z * a1.acceleration.z)) * RAD_TO_DEG;
        rollAcc2 = atan2(-a2.acceleration.y, sqrt(a2.acceleration.x * a2.acceleration.x + a2.acceleration.z * a2.acceleration.z)) * RAD_TO_DEG;
        pitchAcc1 = atan2(-a1.acceleration.x, sqrt(a1.acceleration.y * a1.acceleration.y + a1.acceleration.z * a1.acceleration.z)) * RAD_TO_DEG;
        pitchAcc2 = atan2(-a2.acceleration.x, sqrt(a2.acceleration.y * a2.acceleration.y + a2.acceleration.z * a2.acceleration.z)) * RAD_TO_DEG;

        roll1 = kalman_Roll1.update(rollAcc1, g1.gyro.x * RAD_TO_DEG, dt/1000);
        roll2 = kalman_Roll2.update(rollAcc2, g2.gyro.x * RAD_TO_DEG, dt/1000);
        pitch1 = kalman_Pitch1.update(pitchAcc1, -g1.gyro.y * RAD_TO_DEG, dt/1000);
        pitch2 = kalman_Pitch2.update(pitchAcc2, -g2.gyro.y * RAD_TO_DEG, dt/1000);
        yaw1 = kalman_Yaw1.update(0.0, g1.gyro.z * RAD_TO_DEG, dt/1000);
        yaw2 = kalman_Yaw2.update(0.0, g2.gyro.z * RAD_TO_DEG, dt/1000);

        omega1 = PLL(pitch1, &omega_hat1, &phi_hat1, dt);
        omega2 = PLL(pitch2, &omega_hat2, &phi_hat2, dt);

        logData[bufferCount].time_ms = now_time;
        logData[bufferCount].m1_deg = roll1;
        logData[bufferCount].m1_cur = pitch1;
        logData[bufferCount].m1_torq = yaw1;
        logData[bufferCount].m2_deg = omega1;
        logData[bufferCount].m2_cur = roll2;
        logData[bufferCount].m2_torq = pitch2;
        logData[bufferCount].m3_deg = yaw2;
        logData[bufferCount].m3_cur = omega2;
        
        bufferCount++;

        if (bufferCount >= BUFFER_SIZE) {
            xQueueSend(sdQueue, logData, portMAX_DELAY);
            bufferCount = 0;
        }

        canvas.fillScreen(BLACK);
        canvas.setTextSize(2);
        canvas.setTextColor(WHITE,BLACK);
        canvas.setCursor(0,0);
        canvas.print(dt);
        canvas.setTextSize(3);
        if(pitch1 < 0.0){
            canvas.setTextColor(GREEN,BLACK);
        }else{
            canvas.setTextColor(RED,BLACK);
        }
        canvas.drawCenterString(String(pitch1,3), M5.Display.width() / 2, M5.Display.height() / 2 -20);
        if(pitch2 > 0.0){
            canvas.setTextColor(GREEN,BLACK);
        }else{
            canvas.setTextColor(RED,BLACK);
        }
        canvas.drawCenterString(String(pitch2,3), M5.Display.width() / 2, M5.Display.height() / 2 +20);
        canvas.pushSprite(0, 0);
        // Serial.println(dt);
    }
    timerAlarmDisable(timer);
    xQueueSend(sdQueue, logData, portMAX_DELAY);

    delay(1000);
    logFile.close();

    display_clear();
    display_menu(); 
}

void pace_FixCycle(void){
    const float front_deg = 25;
    const float front_deg_range = 3;

    float roll1,pitch1,yaw1;
    float roll2,pitch2,yaw2;
    SimpleKalmanFilter kalmanPitch1;
    SimpleKalmanFilter kalmanPitch2;
    sensors_event_t a1, mag1, g1, temp1;
    sensors_event_t a2, mag2, g2, temp2; 
    float pitchAcc1, pitchAcc2;

    const int control_cycle = 5000; //制御周期(us)
    const int gear_rate = 16;
    const float max_degree = radians(10);
    float sine_fleq = 0.765;//0.679; //sine波周波数(Hz)

    float degree_1,degree_2,degree_3,degree_4;
    float old_degree = 0.0;
    float time = 0.0;

    boolean mainmode_flag = false;

    display_clear();
    display_println("main");

    const int BUFFER_SIZE = 10;
    // String dataBuffer = "";  // 書き込み用バッファ
    int bufferCount = 0;     // バッファ内データ件数
    // float m1deg,m2deg,m3deg,m4deg = 0.0;
    // float m1cur,m2cur,m3cur,m4cur = 0.0;

    LogData logData[BUFFER_SIZE] = {0};

    if (!SD.begin(4)) {
        Serial.println("SDカードの初期化に失敗しました");
        while (1);
    }

    String filename = makeUniqueFilename("/pace_fixCycle.csv");
    logFile = SD.open(filename, FILE_WRITE);
    if (!logFile) {
        Serial.println("ファイルオープン失敗");
        while (1);
    }

    // ヘッダー
    logFile.println("time_ms,m1_deg,m1_cur,m2_deg,m2_cur,m3_deg,m3_cur,m4_deg,m4_cur,front,back");
    logFile.flush();

    // ドライバーリセット
    for(int i = 1; i <= 4; i++){
        tx_message.identifier = 0x100 + i;
        tx_message.data[0] = 'R';
        tx_message.data_length_code = 1;
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
        int count = 0;
        boolean mainmode_flag = false;
        do{
            delay(1);
            count++;
            if (count > 8000) { // タイムアウト処理
                Serial.print("timeout");
                mainmode_flag = true;
            }
            if(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
                if(rx_message.identifier == 0x200 + i && rx_message.data[0] == 'R'){
                    mainmode_flag = true;
                }
            }
        }while(mainmode_flag == false);
        Serial.printf("%#x:reset\n",0x100+i);
    }

    // キャリブレーション
    for(int i = 1; i <= 4; i++){
        tx_message.identifier = 0x100 + i;
        tx_message.data[0] = 'L';
        tx_message.data_length_code = 1;
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
        int count = 0;
        mainmode_flag = false;
        do{
            delay(1);
            count++;
            if (count > 5000) { // タイムアウト処理
                Serial.print("timeout");
                mainmode_flag = true;
            }
            if(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
                if(rx_message.identifier == 0x200 + i && rx_message.data[0] == 'L'){
                    mainmode_flag = true;
                }
            }
        }while(mainmode_flag == false);
        Serial.printf("%#x:calib\n",0x100+i);
    }

    unsigned long lastTime = millis();
    int count = 0;
    while(count < 50){    //一定角度で安定するまで待機
        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0f;
        lastTime = now;
        lsm1.getEvent(&a1, &mag1, &g1, &temp1);
        lsm2.getEvent(&a2, &mag2, &g2, &temp2);
        pitchAcc1 = atan2(-a1.acceleration.x, sqrt(a1.acceleration.y * a1.acceleration.y + a1.acceleration.z * a1.acceleration.z)) * RAD_TO_DEG;
        pitchAcc2 = atan2(-a2.acceleration.x, sqrt(a2.acceleration.y * a2.acceleration.y + a2.acceleration.z * a2.acceleration.z)) * RAD_TO_DEG;
        pitch1 = kalmanPitch1.update(pitchAcc1, -g1.gyro.y * RAD_TO_DEG, dt);
        pitch2 = kalmanPitch2.update(pitchAcc2, -g2.gyro.y * RAD_TO_DEG, dt);
        if(pitch1 <= front_deg + front_deg_range && pitch1 >= front_deg - front_deg_range && pitch2 <= -front_deg + front_deg_range && pitch2 >= -front_deg - front_deg_range){
            count++;
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.setTextColor(GREEN,BLACK);
            canvas.drawCenterString(String(pitch1,3), M5.Display.width() / 2, M5.Display.height() / 2 -20);
            canvas.drawCenterString(String(pitch2,3), M5.Display.width() / 2, M5.Display.height() / 2 +20);
            canvas.pushSprite(0, 0);
        }else{
            count = 0;
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.drawCenterString(String(pitch1,3), M5.Display.width() / 2, M5.Display.height() / 2 -20);
            canvas.drawCenterString(String(pitch2,3), M5.Display.width() / 2, M5.Display.height() / 2 +20);
            canvas.pushSprite(0, 0);
        }
    }
    Serial.println("degree ready");
// */
    while(pitch1 >= 0){    //0°を跨ぐまで待機
        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0f;
        lastTime = now;
        lsm1.getEvent(&a1, &mag1, &g1, &temp1);
        lsm2.getEvent(&a2, &mag2, &g2, &temp2);
        pitchAcc1 = atan2(-a1.acceleration.x, sqrt(a1.acceleration.y * a1.acceleration.y + a1.acceleration.z * a1.acceleration.z)) * RAD_TO_DEG;
        pitchAcc2 = atan2(-a2.acceleration.x, sqrt(a2.acceleration.y * a2.acceleration.y + a2.acceleration.z * a2.acceleration.z)) * RAD_TO_DEG;
        pitch1 = kalmanPitch1.update(pitchAcc1, -g1.gyro.y * RAD_TO_DEG, dt);
        pitch2 = kalmanPitch2.update(pitchAcc2, -g2.gyro.y * RAD_TO_DEG, dt);
        if(pitch1 <= front_deg + front_deg_range && pitch1 >= front_deg - front_deg_range && pitch2 <= -front_deg + front_deg_range && pitch2 >= -front_deg - front_deg_range){
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.setCursor(0,20);
            canvas.print("degree ready");
            canvas.setTextColor(GREEN,BLACK);
            canvas.drawCenterString(String(pitch1,3), M5.Display.width() / 2, M5.Display.height() / 2 -20);
            canvas.drawCenterString(String(pitch2,3), M5.Display.width() / 2, M5.Display.height() / 2 +20);
            canvas.pushSprite(0, 0);
        }else{
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.setCursor(0,20);
            canvas.print("degree ready");
            canvas.drawCenterString(String(pitch1,3), M5.Display.width() / 2, M5.Display.height() / 2 -20);
            canvas.drawCenterString(String(pitch2,3), M5.Display.width() / 2, M5.Display.height() / 2 +20);
            canvas.pushSprite(0, 0);
        }
    }
    Serial.println("GO");

    lsm1.getEvent(&a1, &mag1, &g1, &temp1);
    lsm2.getEvent(&a2, &mag2, &g2, &temp2);
    if(-g1.gyro.y > 0){
        while(-g1.gyro.y >= 0){
            unsigned long now = millis();
            float dt = (now - lastTime) / 1000.0f;
            lastTime = now;
            lsm1.getEvent(&a1, &mag1, &g1, &temp1);
            lsm2.getEvent(&a2, &mag2, &g2, &temp2);
            pitchAcc1 = atan2(-a1.acceleration.x, sqrt(a1.acceleration.y * a1.acceleration.y + a1.acceleration.z * a1.acceleration.z)) * RAD_TO_DEG;
            pitchAcc2 = atan2(-a2.acceleration.x, sqrt(a2.acceleration.y * a2.acceleration.y + a2.acceleration.z * a2.acceleration.z)) * RAD_TO_DEG;
            pitch1 = kalmanPitch1.update(pitchAcc1, -g1.gyro.y * RAD_TO_DEG, dt);
            pitch2 = kalmanPitch2.update(pitchAcc2, -g2.gyro.y * RAD_TO_DEG, dt);
        }
    }else{
        while(-g1.gyro.y <= 0){
            unsigned long now = millis();
            float dt = (now - lastTime) / 1000.0f;
            lastTime = now;
            lsm1.getEvent(&a1, &mag1, &g1, &temp1);
            lsm2.getEvent(&a2, &mag2, &g2, &temp2);
            pitchAcc1 = atan2(-a1.acceleration.x, sqrt(a1.acceleration.y * a1.acceleration.y + a1.acceleration.z * a1.acceleration.z)) * RAD_TO_DEG;
            pitchAcc2 = atan2(-a2.acceleration.x, sqrt(a2.acceleration.y * a2.acceleration.y + a2.acceleration.z * a2.acceleration.z)) * RAD_TO_DEG;
            pitch1 = kalmanPitch1.update(pitchAcc1, -g1.gyro.y * RAD_TO_DEG, dt);
            pitch2 = kalmanPitch2.update(pitchAcc2, -g2.gyro.y * RAD_TO_DEG, dt);
        }
    }

    for(int i = 1; i <= 4; i++){
        tx_message.identifier = 0x100 + i;
        tx_message.data[0] = 'M';
        tx_message.data[1] = 'T';
        ((float*)&tx_message.data[2])[0] = 0.0;
        tx_message.data_length_code = 2 + sizeof(float); // 6
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
    }
    
    count = 0;

    SD_setup(BUFFER_SIZE);

    // 割り込み設定
    // Initialize intrupt
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, control_cycle, true);
    timerAlarmEnable(timer);

    while(true){
        // 割り込みにより200Hzで以降のコードが実行
        while (Loop_flag == 0);
        Loop_flag = 0;
        M5.update();
        if(M5.BtnA.isPressed()){
            break;
        }

        lsm1.getEvent(&a1, &mag1, &g1, &temp1);
        lsm2.getEvent(&a2, &mag2, &g2, &temp2);
        pitchAcc1 = atan2(-a1.acceleration.x, sqrt(a1.acceleration.y * a1.acceleration.y + a1.acceleration.z * a1.acceleration.z)) * RAD_TO_DEG;
        pitchAcc2 = atan2(-a2.acceleration.x, sqrt(a2.acceleration.y * a2.acceleration.y + a2.acceleration.z * a2.acceleration.z)) * RAD_TO_DEG;
        pitch1 = kalmanPitch1.update(pitchAcc1, -g1.gyro.y * RAD_TO_DEG, 0.005);
        pitch2 = kalmanPitch2.update(pitchAcc2, -g2.gyro.y * RAD_TO_DEG, 0.005);
        //sine_fleq = estimatePeriod(pitch, gyroY);

        time += control_cycle/1000;
        degree_1 = -max_degree * sin(2*PI*sine_fleq*time/1000 + 0);
        degree_2 = max_degree * sin(2*PI*sine_fleq*time/1000 + PI);
        degree_3 = -max_degree * sin(2*PI*sine_fleq*time/1000 + 0);
        degree_4 = max_degree * sin(2*PI*sine_fleq*time/1000 + PI);

        tx_message.identifier = 0x101;
        tx_message.data[0] = 'P';
        ((float*)&tx_message.data[1])[0] = degree_1;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));
        //ets_delay_us(200);  // 200µs

        tx_message.identifier = 0x102;
        tx_message.data[0] = 'P';
        ((float*)&tx_message.data[1])[0] = degree_2;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));
        //ets_delay_us(200);  // 200µs

        tx_message.identifier = 0x103;
        tx_message.data[0] = 'P';
        ((float*)&tx_message.data[1])[0] = degree_3;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));
        //ets_delay_us(200);  // 200µs

        tx_message.identifier = 0x104;
        tx_message.data[0] = 'P';
        ((float*)&tx_message.data[1])[0] = degree_4;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));
        //ets_delay_us(200);  // 200µs

        while(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
            if(rx_message.identifier == 0x201){
                logData[bufferCount].m1_deg = *((float*)&rx_message.data[0]);
                logData[bufferCount].m1_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x202){
                logData[bufferCount].m2_deg = *((float*)&rx_message.data[0]);
                logData[bufferCount].m2_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x203){
                logData[bufferCount].m3_deg = *((float*)&rx_message.data[0]);
                logData[bufferCount].m3_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x204){
                logData[bufferCount].m4_deg = *((float*)&rx_message.data[0]);
                logData[bufferCount].m4_cur = *((float*)&rx_message.data[4]);
            }
        }

        logData[bufferCount].time_ms = millis();
        logData[bufferCount].front = pitch1;
        logData[bufferCount].back = pitch2;
        
        bufferCount++;

        if (bufferCount >= BUFFER_SIZE) {
            xQueueSend(sdQueue, logData, portMAX_DELAY);
            bufferCount = 0;
        }

        old_degree = degree_1;
    }
    timerAlarmDisable(timer);

    xQueueSend(sdQueue, logData, portMAX_DELAY);

    tx_message.identifier = 0x101;
    tx_message.data[0] = 'T';
    ((float*)&tx_message.data[1])[0] = 0.0;
    tx_message.data_length_code = 1 + sizeof(float); // 5
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    tx_message.identifier = 0x102;
    tx_message.data[0] = 'T';
    ((float*)&tx_message.data[1])[0] = 0.0;
    tx_message.data_length_code = 1 + sizeof(float); // 5
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    tx_message.identifier = 0x103;
    tx_message.data[0] = 'T';
    ((float*)&tx_message.data[1])[0] = 0.0;
    tx_message.data_length_code = 1 + sizeof(float); // 5
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    tx_message.identifier = 0x104;
    tx_message.data[0] = 'T';
    ((float*)&tx_message.data[1])[0] = 0.0;
    tx_message.data_length_code = 1 + sizeof(float); // 5
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    delay(1000);
    logFile.close();

    ESP.restart();
}

void pace_SimpleVariationCycle(void){
    const float front_deg = 20;
    const float front_deg_range = 3;

    float roll,pitch,yaw;
    float old_pitch = 0.0;

    const int control_cycle = 5000; //制御周期(us)
    const int gear_rate = 16;
    const float max_degree = radians(15);
    float sine_fleq = 0.85;//0.679; //sine波周波数(Hz)

    float degree_1,degree_2,degree_3,degree_4;
    float old_degree = 0.0;
    float time = 0.0;

    boolean mainmode_flag = false;

    display_clear();
    display_println("main");

    const int BUFFER_SIZE = 10;
    // String dataBuffer = "";  // 書き込み用バッファ
    int bufferCount = 0;     // バッファ内データ件数
    // float m1deg,m2deg,m3deg,m4deg = 0.0;
    // float m1cur,m2cur,m3cur,m4cur = 0.0;

    LogData logData[BUFFER_SIZE] = {0};

    if (!SD.begin(4)) {
        Serial.println("SDカードの初期化に失敗しました");
        while (1);
    }

    String filename = makeUniqueFilename("/pace_simVari.csv");
    logFile = SD.open(filename, FILE_WRITE);
    if (!logFile) {
        Serial.println("ファイルオープン失敗");
        while (1);
    }

    // ヘッダー
    logFile.println("time_ms,m1_deg,m1_cur,m2_deg,m2_cur,m3_deg,m3_cur,m4_deg,m4_cur,front");
    logFile.flush();

    // ドライバーリセット
    for(int i = 1; i <= 4; i++){
        tx_message.identifier = 0x100 + i;
        tx_message.data[0] = 'R';
        tx_message.data_length_code = 1;
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
        int count = 0;
        boolean mainmode_flag = false;
        do{
            delay(1);
            count++;
            if (count > 8000) { // タイムアウト処理
                Serial.print("timeout");
                mainmode_flag = true;
            }
            if(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
                if(rx_message.identifier == 0x200 + i && rx_message.data[0] == 'R'){
                    mainmode_flag = true;
                }
            }
        }while(mainmode_flag == false);
        Serial.printf("%#x:reset\n",0x100+i);
    }

    // キャリブレーション
    for(int i = 1; i <= 4; i++){
        tx_message.identifier = 0x100 + i;
        tx_message.data[0] = 'L';
        tx_message.data_length_code = 1;
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
        int count = 0;
        mainmode_flag = false;
        do{
            delay(1);
            count++;
            if (count > 5000) { // タイムアウト処理
                Serial.print("timeout");
                mainmode_flag = true;
            }
            if(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
                if(rx_message.identifier == 0x200 + i && rx_message.data[0] == 'L'){
                    mainmode_flag = true;
                }
            }
        }while(mainmode_flag == false);
        Serial.printf("%#x:calib\n",0x100+i);
    }

    int count = 0;
    while(count < 50){    //一定角度で安定するまで待機
        kalman_update(&roll,&pitch,&yaw);
        if(pitch <= front_deg + front_deg_range && pitch >= front_deg - front_deg_range){
            count++;
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.setTextColor(GREEN,BLACK);
            canvas.drawCenterString(String(pitch,3), M5.Display.width() / 2, M5.Display.height() / 2);
            canvas.pushSprite(0, 0);
        }else{
            count = 0;
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.drawCenterString(String(pitch,3), M5.Display.width() / 2, M5.Display.height() / 2);
            canvas.pushSprite(0, 0);
        }
    }
    Serial.println("degree ready");
    // */
    while(pitch >= 0){    //0°を跨ぐまで待機
        kalman_update(&roll,&pitch,&yaw);
        if(pitch <= front_deg + front_deg_range && pitch >= front_deg - front_deg_range){
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.setCursor(0,20);
            canvas.print("degree ready");
            canvas.setTextColor(GREEN,BLACK);
            canvas.drawCenterString(String(pitch,3), M5.Display.width() / 2, M5.Display.height() / 2);
            canvas.pushSprite(0, 0);
        }else{
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.setCursor(0,20);
            canvas.print("degree ready");
            canvas.drawCenterString(String(pitch,3), M5.Display.width() / 2, M5.Display.height() / 2);
            canvas.pushSprite(0, 0);
        }
    }
    Serial.println("GO");

    float gyroX, gyroY, gyroZ;
    M5.update();
    M5.Imu.getGyroData(&gyroX, &gyroY, &gyroZ);
    if(gyroY > 0){
        while(gyroY >= 0){
            M5.update();
            kalman_update(&roll,&pitch,&yaw);
            M5.Imu.getGyroData(&gyroX, &gyroY, &gyroZ);
            //sine_fleq = estimatePeriod(pitch, gyroY);
        }
    }else{
        while(gyroY <= 0){
            M5.update();
            kalman_update(&roll,&pitch,&yaw);
            M5.Imu.getGyroData(&gyroX, &gyroY, &gyroZ);
            //sine_fleq = estimatePeriod(pitch, gyroY);
        }
    }
    
    count = 0;

    SD_setup(BUFFER_SIZE);

    // 割り込み設定
    // Initialize intrupt
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, control_cycle, true);
    timerAlarmEnable(timer);

    while(true){
        // 割り込みにより200Hzで以降のコードが実行
        while (Loop_flag == 0);
        Loop_flag = 0;
        M5.update();
        if(M5.BtnA.isPressed()){
            break;
        }

        kalman_update(&roll,&pitch,&yaw);
        //M5.Imu.getGyroData(&gyroX, &gyroY, &gyroZ);
        //sine_fleq = estimatePeriod(pitch, gyroY);

        time += control_cycle/1000;
        degree_1 = max_degree * sin(2*PI*sine_fleq*time/1000 + 0) - radians(5);

        // canvas.fillScreen(BLACK);
        // canvas.setTextSize(2);
        // canvas.setTextColor(WHITE,BLACK);
        // canvas.setCursor(0,0);
        // canvas.print("main");
        // canvas.setTextSize(3);
        // canvas.drawCenterString(String(pitch,3), M5.Display.width() / 2, M5.Display.height() / 2);
        // canvas.drawCenterString(String(degree_1,3), M5.Display.width() / 2, M5.Display.height() / 2 + 30);
        // canvas.pushSprite(0, 0);

        if(degree_1 > 0.0 && degree_1 < old_degree){ //上昇から下降に変わるタイミング
            if(old_pitch < -2.0 && pitch < 2.0){   //脚入れ替え前
                time -= control_cycle/1000; //時間を戻す
                degree_1 = old_degree;
            }
        }else if(degree_1 < 0.0 && degree_1 > old_degree){
            if(old_pitch > 2.0 && pitch > -2.0){
                time -= control_cycle/1000;
                degree_1 = old_degree;
            }
        }
        degree_2 = -max_degree * sin(2*PI*sine_fleq*time/1000 + PI) + radians(5);
        degree_3 = max_degree * sin(2*PI*sine_fleq*time/1000 + 0) - radians(5);
        degree_4 = -max_degree * sin(2*PI*sine_fleq*time/1000 + PI) + radians(5);

        tx_message.identifier = 0x101;
        tx_message.data[0] = 'P';
        ((float*)&tx_message.data[1])[0] = degree_1;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));
        //ets_delay_us(200);  // 200µs

        tx_message.identifier = 0x102;
        tx_message.data[0] = 'P';
        ((float*)&tx_message.data[1])[0] = degree_2;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));
        //ets_delay_us(200);  // 200µs

        tx_message.identifier = 0x103;
        tx_message.data[0] = 'P';
        ((float*)&tx_message.data[1])[0] = degree_3;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));
        //ets_delay_us(200);  // 200µs

        tx_message.identifier = 0x104;
        tx_message.data[0] = 'P';
        ((float*)&tx_message.data[1])[0] = degree_4;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));
        //ets_delay_us(200);  // 200µs

        while(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
            if(rx_message.identifier == 0x201){
                logData[bufferCount].m1_deg = *((float*)&rx_message.data[0]);
                logData[bufferCount].m1_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x202){
                logData[bufferCount].m2_deg = *((float*)&rx_message.data[0]);
                logData[bufferCount].m2_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x203){
                logData[bufferCount].m3_deg = *((float*)&rx_message.data[0]);
                logData[bufferCount].m3_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x204){
                logData[bufferCount].m4_deg = *((float*)&rx_message.data[0]);
                logData[bufferCount].m4_cur = *((float*)&rx_message.data[4]);
            }
        }

        // if(count == 1 || count == 3 || count == 5 || count == 7 || count == 8) count++;
        //if(count != 0) count++;

        //if(count == 0){
            logData[bufferCount].time_ms = millis();
            logData[bufferCount].front = pitch;

            // dataBuffer += String(time_millis);
            // dataBuffer += ",";
            // dataBuffer += String(m1deg, 3);
            // dataBuffer += ",";
            // dataBuffer += String(m1cur, 3);
            // dataBuffer += ",";
            // dataBuffer += String(m2deg, 3);
            // dataBuffer += ",";
            // dataBuffer += String(m2cur, 3);
            // dataBuffer += ",";
            // dataBuffer += String(m3deg, 3);
            // dataBuffer += ",";
            // dataBuffer += String(m3cur, 3);
            // dataBuffer += ",";
            // dataBuffer += String(m4deg, 3);
            // dataBuffer += ",";
            // dataBuffer += String(m4cur, 3);
            // dataBuffer += ",";
            // dataBuffer += String(pitch, 3);
            // dataBuffer += "\n";

            bufferCount++;

            // logFile.print(dataBuffer);
            // logFile.flush(); // 一括保存
            // dataBuffer = ""; // バッファクリア
        //}

        if (bufferCount >= BUFFER_SIZE) {
            // 10件たまったので書き出し
            // logFile.print(dataBuffer);
            // logFile.flush(); // 一括保存
            // dataBuffer = ""; // バッファクリア
            // flush_flag = true;
            xQueueSend(sdQueue, logData, portMAX_DELAY);
            bufferCount = 0;
        }

        old_pitch = pitch;
        old_degree = degree_1;
    }
    timerAlarmDisable(timer);

    xQueueSend(sdQueue, logData, portMAX_DELAY);

    tx_message.identifier = 0x101;
    tx_message.data[0] = 'P';
    ((float*)&tx_message.data[1])[0] = 0.0;
    tx_message.data_length_code = 1 + sizeof(float); // 5
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    tx_message.identifier = 0x102;
    tx_message.data[0] = 'P';
    ((float*)&tx_message.data[1])[0] = 0.0;
    tx_message.data_length_code = 1 + sizeof(float); // 5
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    tx_message.identifier = 0x103;
    tx_message.data[0] = 'P';
    ((float*)&tx_message.data[1])[0] = 0.0;
    tx_message.data_length_code = 1 + sizeof(float); // 5
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    tx_message.identifier = 0x104;
    tx_message.data[0] = 'P';
    ((float*)&tx_message.data[1])[0] = 0.0;
    tx_message.data_length_code = 1 + sizeof(float); // 5
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    delay(1000);
    logFile.close();

    ESP.restart();
}

void pace_VariationCycle(void){

}

void pace_VirtualPassiveBase(void){
    const float front_deg = 20;
    const float front_deg_range = 3;

    float roll,pitch,yaw;
    float old_pitch = 0.0;

    const float tan_phi = tan(radians(5)); //仮想重力の傾きtanφ

    const int control_cycle = 5000; //制御周期(us)
    const int gear_rate = 16;
    const float KT = 0.150; //トルク定数(Nm/A)
    const float L1 = 0.3;
    const float m1 = 0.564;
    const float m2 = 0.378;
    const float m3 = 0.735;
    const float g = 9.81;
    const float u1 = 0.7f * L1 * (3*m1 + 4*m2 + m3) * g * tan_phi / KT / gear_rate;
    const float u2 = 1.5f * m1 * L1 * g * tan_phi / KT / gear_rate;
    float degree_1 = 0.0f, degree_2 = 0.0f, degree_3 = 0.0f, degree_4 = 0.0f;
    float torque_1 = 0.0f, torque_2 = 0.0f, torque_3 = 0.0f, torque_4 = 0.0f;
    float time = 0.0;

    float torque_limit = 0.85f;
    float degree_limit = 15.0f;

    boolean mainmode_flag = false;

    display_clear();
    display_println("main");

    const int BUFFER_SIZE = 10;
    int bufferCount = 0;     // バッファ内データ件数

    LogData logData[BUFFER_SIZE] = {0};

    if (!SD.begin(4)) {
        Serial.println("SDカードの初期化に失敗しました");
        while (1);
    }

    String filename = makeUniqueFilename("/pace_VPB.csv");
    logFile = SD.open(filename, FILE_WRITE);
    if (!logFile) {
        Serial.println("ファイルオープン失敗");
        while (1);
    }

    // ヘッダー
    logFile.println("time_ms,m1_deg,m1_cur,m1_torq,m2_deg,m2_cur,m2_torq,m3_deg,m3_cur,m3_torq,m4_deg,m4_cur,m4_torq,front,back");
    logFile.flush();

    // ドライバーリセット
    for(int i = 1; i <= 4; i++){
        tx_message.identifier = 0x100 + i;
        tx_message.data[0] = 'R';
        tx_message.data_length_code = 1;
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
        int count = 0;
        boolean mainmode_flag = false;
        do{
            delay(1);
            count++;
            if (count > 8000) { // タイムアウト処理
                Serial.print("timeout");
                mainmode_flag = true;
            }
            if(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
                if(rx_message.identifier == 0x200 + i && rx_message.data[0] == 'R'){
                    mainmode_flag = true;
                }
            }
        }while(mainmode_flag == false);
        Serial.printf("%#x:reset\n",0x100+i);
    }

    // キャリブレーション
    for(int i = 1; i <= 4; i++){
        tx_message.identifier = 0x100 + i;
        tx_message.data[0] = 'L';
        tx_message.data_length_code = 1;
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
        int count = 0;
        mainmode_flag = false;
        do{
            delay(1);
            count++;
            if (count > 5000) { // タイムアウト処理
                Serial.print("timeout");
                mainmode_flag = true;
            }
            if(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
                if(rx_message.identifier == 0x200 + i && rx_message.data[0] == 'L'){
                    mainmode_flag = true;
                }
            }
        }while(mainmode_flag == false);
        Serial.printf("%#x:calib\n",0x100+i);
    }

    // 制限設定
    for(int i = 1; i <= 4; i++){
        tx_message.identifier = 0x100 + i;
        tx_message.data[0] = 'B';
        tx_message.data[1] = 'A';
        ((float*)&tx_message.data[2])[0] = degree_limit;
        tx_message.data_length_code = 2 + sizeof(float);
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
    }

    int count = 0;
    while(count < 50){    //一定角度で安定するまで待機
        kalman_update(&roll,&pitch,&yaw);
        if(pitch <= front_deg + front_deg_range && pitch >= front_deg - front_deg_range){
            count++;
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.setTextColor(GREEN,BLACK);
            canvas.drawCenterString(String(pitch,3), M5.Display.width() / 2, M5.Display.height() / 2);
            canvas.pushSprite(0, 0);
        }else{
            count = 0;
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.drawCenterString(String(pitch,3), M5.Display.width() / 2, M5.Display.height() / 2);
            canvas.pushSprite(0, 0);
        }
    }
    Serial.println("degree ready");
// */
    while(pitch >= 0){    //0°を跨ぐまで待機
        kalman_update(&roll,&pitch,&yaw);
        if(pitch <= front_deg + front_deg_range && pitch >= front_deg - front_deg_range){
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.setCursor(0,20);
            canvas.print("degree ready");
            canvas.setTextColor(GREEN,BLACK);
            canvas.drawCenterString(String(pitch,3), M5.Display.width() / 2, M5.Display.height() / 2);
            canvas.pushSprite(0, 0);
        }else{
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.setCursor(0,20);
            canvas.print("degree ready");
            canvas.drawCenterString(String(pitch,3), M5.Display.width() / 2, M5.Display.height() / 2);
            canvas.pushSprite(0, 0);
        }
    }
    Serial.println("GO");

    // float gyroX, gyroY, gyroZ;
    // M5.update();
    // M5.Imu.getGyroData(&gyroX, &gyroY, &gyroZ);
    // if(gyroY > 0){
    //     while(gyroY >= 0){
    //         M5.update();
    //         kalman_update(&roll,&pitch,&yaw);
    //         M5.Imu.getGyroData(&gyroX, &gyroY, &gyroZ);
    //         //sine_fleq = estimatePeriod(pitch, gyroY);
    //     }
    // }else{
    //     while(gyroY <= 0){
    //         M5.update();
    //         kalman_update(&roll,&pitch,&yaw);
    //         M5.Imu.getGyroData(&gyroX, &gyroY, &gyroZ);
    //         //sine_fleq = estimatePeriod(pitch, gyroY);
    //     }
    // }

    for(int i = 1; i <= 4; i++){
        tx_message.identifier = 0x100 + i;
        tx_message.data[0] = 'M';
        tx_message.data[1] = 'T';
        ((float*)&tx_message.data[2])[0] = 0.0;
        tx_message.data_length_code = 2 + sizeof(float); // 6
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
    }
    
    count = 0;

    SD_setup(BUFFER_SIZE);

    // 割り込み設定
    // Initialize intrupt
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, control_cycle, true);
    timerAlarmEnable(timer);

    while(true){
        // 割り込みにより200Hzで以降のコードが実行
        while (Loop_flag == 0);
        Loop_flag = 0;
        M5.update();
        if(M5.BtnA.isPressed()){
            break;
        }

        kalman_update(&roll,&pitch,&yaw);
        //M5.Imu.getGyroData(&gyroX, &gyroY, &gyroZ);
        //sine_fleq = estimatePeriod(pitch, gyroY);

        time += control_cycle/1000;

        if(pitch > 1.0){   //機体がどちらに傾いているか（どちらの脚をついているか）
            torque_1 = -u1 * cos(degree_1) * abs(degree_3 / (abs(degree_1) + abs(degree_3)));
            torque_2 = -u2 * cos(degree_2);
            torque_3 = -u1 * cos(degree_3) * abs(degree_1 / (abs(degree_3) + abs(degree_1)));
            torque_4 = -u2 * cos(degree_4);
        }else if(pitch < -1.0){
            torque_1 = u2 * cos(degree_1);
            torque_2 = u1 * cos(degree_2) * abs(degree_4 / (abs(degree_2) + abs(degree_4)));
            torque_3 = u2 * cos(degree_3);
            torque_4 = u1 * cos(degree_4) * abs(degree_2 / (abs(degree_4) + abs(degree_2)));
        }else{
            torque_1 = 0.0;
            torque_2 = 0.0;
            torque_3 = 0.0;
            torque_4 = 0.0;
        }

        if(isfinite(torque_1) == false) torque_1 = 0.0;
        if(isfinite(torque_2) == false) torque_2 = 0.0;
        if(isfinite(torque_3) == false) torque_3 = 0.0;
        if(isfinite(torque_4) == false) torque_4 = 0.0;

        tx_message.identifier = 0x101;
        tx_message.data[0] = 'T';
        ((float*)&tx_message.data[1])[0] = torque_1;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));
        //ets_delay_us(200);  // 200µs

        tx_message.identifier = 0x102;
        tx_message.data[0] = 'T';
        ((float*)&tx_message.data[1])[0] = torque_2;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));
        //ets_delay_us(200);  // 200µs

        tx_message.identifier = 0x103;
        tx_message.data[0] = 'T';
        ((float*)&tx_message.data[1])[0] = torque_3;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));
        //ets_delay_us(200);  // 200µs

        tx_message.identifier = 0x104;
        tx_message.data[0] = 'T';
        ((float*)&tx_message.data[1])[0] = torque_4;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));
        //ets_delay_us(200);  // 200µs

        logData[bufferCount] = {0};

        logData[bufferCount].m1_torq = torque_1;
        logData[bufferCount].m2_torq = torque_2;
        logData[bufferCount].m3_torq = torque_3;
        logData[bufferCount].m4_torq = torque_4;

        while(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
            if(rx_message.identifier == 0x201){
                logData[bufferCount].m1_deg = *((float*)&rx_message.data[0]);
                degree_1 = logData[bufferCount].m1_deg;
                logData[bufferCount].m1_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x202){
                logData[bufferCount].m2_deg = *((float*)&rx_message.data[0]);
                degree_2 = logData[bufferCount].m2_deg;
                logData[bufferCount].m2_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x203){
                logData[bufferCount].m3_deg = *((float*)&rx_message.data[0]);
                degree_3 = logData[bufferCount].m3_deg;
                logData[bufferCount].m3_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x204){
                logData[bufferCount].m4_deg = *((float*)&rx_message.data[0]);
                degree_4 = logData[bufferCount].m4_deg;
                logData[bufferCount].m4_cur = *((float*)&rx_message.data[4]);
            }
        }

        logData[bufferCount].time_ms = millis();
        logData[bufferCount].front = radians(pitch);

        bufferCount++;

        if (bufferCount >= BUFFER_SIZE) {
            xQueueSend(sdQueue, logData, portMAX_DELAY);
            bufferCount = 0;
        }

        old_pitch = pitch;
    }
    timerAlarmDisable(timer);

    xQueueSend(sdQueue, logData, portMAX_DELAY);

    tx_message.identifier = 0x101;
    tx_message.data[0] = 'E';
    tx_message.data[1] = '0';
    tx_message.data_length_code = 2;
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    tx_message.identifier = 0x102;
    tx_message.data[0] = 'E';
    tx_message.data[1] = '0';
    tx_message.data_length_code = 2;
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    tx_message.identifier = 0x103;
    tx_message.data[0] = 'E';
    tx_message.data[1] = '0';
    tx_message.data_length_code = 2;
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    tx_message.identifier = 0x104;
    tx_message.data[0] = 'E';
    tx_message.data[1] = '0';
    tx_message.data_length_code = 2;
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    delay(1000);
    logFile.close();

    ESP.restart();
}

void pace_VirtualPassiveBase_rev(void){
    const float front_deg = 20;
    const float front_deg_range = 3;

    float roll,pitch,yaw;
    float old_pitch = 0.0;

    const float tan_phi = tan(radians(5)); //仮想重力の傾きtanφ

    const int control_cycle = 5000; //制御周期(us)
    const int gear_rate = 16;
    const float KT = 0.150; //トルク定数(Nm/A)
    const float L1 = 0.3;
    const float m1 = 0.564;
    const float m2 = 0.378;
    const float m3 = 0.735;
    const float g = 9.81;
    const float u1 = 0.3f * L1 * (3*m1 + 4*m2 + m3) * g * tan_phi / KT / gear_rate;
    const float u2 = 2.5f * m1 * L1 * g * tan_phi / KT / gear_rate;
    float degree_1 = 0.0f, degree_2 = 0.0f, degree_3 = 0.0f, degree_4 = 0.0f;
    float torque_1 = 0.0f, torque_2 = 0.0f, torque_3 = 0.0f, torque_4 = 0.0f;
    float time = 0.0;

    float torque_limit = 0.85f;
    float degree_limit = 15.0f;

    boolean mainmode_flag = false;

    display_clear();
    display_println("main");

    const int BUFFER_SIZE = 10;
    int bufferCount = 0;     // バッファ内データ件数

    LogData logData[BUFFER_SIZE] = {0};

    if (!SD.begin(4)) {
        Serial.println("SDカードの初期化に失敗しました");
        while (1);
    }

    String filename = makeUniqueFilename("/pace_VPB_rev.csv");
    logFile = SD.open(filename, FILE_WRITE);
    if (!logFile) {
        Serial.println("ファイルオープン失敗");
        while (1);
    }

    // ヘッダー
    logFile.println("time_ms,m1_deg,m1_cur,m1_torq,m2_deg,m2_cur,m2_torq,m3_deg,m3_cur,m3_torq,m4_deg,m4_cur,m4_torq,front,back");
    logFile.flush();

    // ドライバーリセット
    for(int i = 1; i <= 4; i++){
        tx_message.identifier = 0x100 + i;
        tx_message.data[0] = 'R';
        tx_message.data_length_code = 1;
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
        int count = 0;
        boolean mainmode_flag = false;
        do{
            delay(1);
            count++;
            if (count > 8000) { // タイムアウト処理
                Serial.print("timeout");
                mainmode_flag = true;
            }
            if(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
                if(rx_message.identifier == 0x200 + i && rx_message.data[0] == 'R'){
                    mainmode_flag = true;
                }
            }
        }while(mainmode_flag == false);
        Serial.printf("%#x:reset\n",0x100+i);
    }

    // キャリブレーション
    for(int i = 1; i <= 4; i++){
        tx_message.identifier = 0x100 + i;
        tx_message.data[0] = 'L';
        tx_message.data_length_code = 1;
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
        int count = 0;
        mainmode_flag = false;
        do{
            delay(1);
            count++;
            if (count > 5000) { // タイムアウト処理
                Serial.print("timeout");
                mainmode_flag = true;
            }
            if(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
                if(rx_message.identifier == 0x200 + i && rx_message.data[0] == 'L'){
                    mainmode_flag = true;
                }
            }
        }while(mainmode_flag == false);
        Serial.printf("%#x:calib\n",0x100+i);
    }

    // 制限設定
    for(int i = 1; i <= 4; i++){
        tx_message.identifier = 0x100 + i;
        tx_message.data[0] = 'B';
        tx_message.data[1] = 'A';
        ((float*)&tx_message.data[2])[0] = degree_limit;
        tx_message.data_length_code = 2 + sizeof(float);
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
    }

    int count = 0;
    while(count < 50){    //一定角度で安定するまで待機
        kalman_update(&roll,&pitch,&yaw);
        if(pitch <= -front_deg + front_deg_range && pitch >= -front_deg - front_deg_range){
            count++;
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.setTextColor(GREEN,BLACK);
            canvas.drawCenterString(String(pitch,3), M5.Display.width() / 2, M5.Display.height() / 2);
            canvas.pushSprite(0, 0);
        }else{
            count = 0;
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.drawCenterString(String(pitch,3), M5.Display.width() / 2, M5.Display.height() / 2);
            canvas.pushSprite(0, 0);
        }
    }
    Serial.println("degree ready");
// */
    while(pitch <= 0){    //0°を跨ぐまで待機
        kalman_update(&roll,&pitch,&yaw);
        if(pitch <= -front_deg + front_deg_range && pitch >= -front_deg - front_deg_range){
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.setCursor(0,20);
            canvas.print("degree ready");
            canvas.setTextColor(GREEN,BLACK);
            canvas.drawCenterString(String(pitch,3), M5.Display.width() / 2, M5.Display.height() / 2);
            canvas.pushSprite(0, 0);
        }else{
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.setCursor(0,20);
            canvas.print("degree ready");
            canvas.drawCenterString(String(pitch,3), M5.Display.width() / 2, M5.Display.height() / 2);
            canvas.pushSprite(0, 0);
        }
    }
    Serial.println("GO");

    // float gyroX, gyroY, gyroZ;
    // M5.update();
    // M5.Imu.getGyroData(&gyroX, &gyroY, &gyroZ);
    // if(gyroY > 0){
    //     while(gyroY >= 0){
    //         M5.update();
    //         kalman_update(&roll,&pitch,&yaw);
    //         M5.Imu.getGyroData(&gyroX, &gyroY, &gyroZ);
    //         //sine_fleq = estimatePeriod(pitch, gyroY);
    //     }
    // }else{
    //     while(gyroY <= 0){
    //         M5.update();
    //         kalman_update(&roll,&pitch,&yaw);
    //         M5.Imu.getGyroData(&gyroX, &gyroY, &gyroZ);
    //         //sine_fleq = estimatePeriod(pitch, gyroY);
    //     }
    // }

    for(int i = 1; i <= 4; i++){
        tx_message.identifier = 0x100 + i;
        tx_message.data[0] = 'M';
        tx_message.data[1] = 'T';
        ((float*)&tx_message.data[2])[0] = 0.0;
        tx_message.data_length_code = 2 + sizeof(float); // 6
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
    }
    
    count = 0;

    SD_setup(BUFFER_SIZE);

    // 割り込み設定
    // Initialize intrupt
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, control_cycle, true);
    timerAlarmEnable(timer);

    while(true){
        // 割り込みにより200Hzで以降のコードが実行
        while (Loop_flag == 0);
        Loop_flag = 0;
        M5.update();
        if(M5.BtnA.isPressed()){
            break;
        }

        kalman_update(&roll,&pitch,&yaw);
        //M5.Imu.getGyroData(&gyroX, &gyroY, &gyroZ);
        //sine_fleq = estimatePeriod(pitch, gyroY);

        time += control_cycle/1000;

        if(pitch > 1.0){   //機体がどちらに傾いているか（どちらの脚をついているか）
            torque_1 = -u1 * cos(degree_1) * abs(degree_3 / (abs(degree_1) + abs(degree_3)));
            torque_2 = -u2 * cos(degree_2) * abs(degree_4 / (abs(degree_2) + abs(degree_4)));
            torque_3 = -u1 * cos(degree_3) * abs(degree_1 / (abs(degree_3) + abs(degree_1)));
            torque_4 = -u2 * cos(degree_4) * abs(degree_2 / (abs(degree_2) + abs(degree_4)));
        }else if(pitch < -1.0){
            torque_1 = u2 * cos(degree_1) * abs(degree_3 / (abs(degree_1) + abs(degree_3)));
            torque_2 = u1 * cos(degree_2) * abs(degree_4 / (abs(degree_2) + abs(degree_4)));
            torque_3 = u2 * cos(degree_3) * abs(degree_1 / (abs(degree_3) + abs(degree_1)));
            torque_4 = u1 * cos(degree_4) * abs(degree_2 / (abs(degree_4) + abs(degree_2)));
        }else{
            torque_1 = 0.0;
            torque_2 = 0.0;
            torque_3 = 0.0;
            torque_4 = 0.0;
        }

        if(isfinite(torque_1) == false) torque_1 = 0.0;
        if(isfinite(torque_2) == false) torque_2 = 0.0;
        if(isfinite(torque_3) == false) torque_3 = 0.0;
        if(isfinite(torque_4) == false) torque_4 = 0.0;

        tx_message.identifier = 0x101;
        tx_message.data[0] = 'T';
        ((float*)&tx_message.data[1])[0] = torque_1;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));
        //ets_delay_us(200);  // 200µs

        tx_message.identifier = 0x102;
        tx_message.data[0] = 'T';
        ((float*)&tx_message.data[1])[0] = torque_2;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));
        //ets_delay_us(200);  // 200µs

        tx_message.identifier = 0x103;
        tx_message.data[0] = 'T';
        ((float*)&tx_message.data[1])[0] = torque_3;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));
        //ets_delay_us(200);  // 200µs

        tx_message.identifier = 0x104;
        tx_message.data[0] = 'T';
        ((float*)&tx_message.data[1])[0] = torque_4;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));
        //ets_delay_us(200);  // 200µs

        logData[bufferCount] = {0};

        logData[bufferCount].m1_torq = torque_1;
        logData[bufferCount].m2_torq = torque_2;
        logData[bufferCount].m3_torq = torque_3;
        logData[bufferCount].m4_torq = torque_4;

        while(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
            if(rx_message.identifier == 0x201){
                logData[bufferCount].m1_deg = *((float*)&rx_message.data[0]);
                degree_1 = logData[bufferCount].m1_deg;
                logData[bufferCount].m1_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x202){
                logData[bufferCount].m2_deg = *((float*)&rx_message.data[0]);
                degree_2 = logData[bufferCount].m2_deg;
                logData[bufferCount].m2_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x203){
                logData[bufferCount].m3_deg = *((float*)&rx_message.data[0]);
                degree_3 = logData[bufferCount].m3_deg;
                logData[bufferCount].m3_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x204){
                logData[bufferCount].m4_deg = *((float*)&rx_message.data[0]);
                degree_4 = logData[bufferCount].m4_deg;
                logData[bufferCount].m4_cur = *((float*)&rx_message.data[4]);
            }
        }

        logData[bufferCount].time_ms = millis();
        logData[bufferCount].front = radians(pitch);

        bufferCount++;

        if (bufferCount >= BUFFER_SIZE) {
            xQueueSend(sdQueue, logData, portMAX_DELAY);
            bufferCount = 0;
        }

        old_pitch = pitch;
    }
    timerAlarmDisable(timer);

    xQueueSend(sdQueue, logData, portMAX_DELAY);

    tx_message.identifier = 0x101;
    tx_message.data[0] = 'E';
    tx_message.data[1] = '0';
    tx_message.data_length_code = 2;
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    tx_message.identifier = 0x102;
    tx_message.data[0] = 'E';
    tx_message.data[1] = '0';
    tx_message.data_length_code = 2;
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    tx_message.identifier = 0x103;
    tx_message.data[0] = 'E';
    tx_message.data[1] = '0';
    tx_message.data_length_code = 2;
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    tx_message.identifier = 0x104;
    tx_message.data[0] = 'E';
    tx_message.data[1] = '0';
    tx_message.data_length_code = 2;
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    delay(1000);
    logFile.close();

    ESP.restart();
}

void pace_VirtualPassiveBase2(void){
    const float front_deg = 30;
    const float front_deg_range = 3;

    float roll1,pitch1,yaw1;
    float roll2,pitch2,yaw2;
    SimpleKalmanFilter kalmanPitch1;
    SimpleKalmanFilter kalmanPitch2;
    sensors_event_t a1, mag1, g1, temp1;
    sensors_event_t a2, mag2, g2, temp2; 
    float pitchAcc1, pitchAcc2;

    float front_fleq,back_fleq;
    float front_omega = 0.0, back_omega = 0.0;
    float front_phi = 0.0, back_phi = 0.0;

    float tan_phi = tan(radians(5)); //仮想重力の傾きtanφ
    float speed = 0.1;

    const int control_cycle = 5000; //制御周期(us)
    const int gear_rate = 16;
    const float max_degree = radians(7.8);
    const float degree_delta = radians(3.0);
    const float KT = 0.150; //トルク定数(Nm/A)
    const float L1 = 0.3;
    const float m1 = 0.564;
    const float m2 = 0.378;
    const float m3 = 0.735;
    const float g = 9.81;
    const float u1 = 1.0f * L1 * (3*m1 + 4*m2 + m3) * g / KT / gear_rate;
    const float u2 = 1.0f * m1 * L1 * g / KT / gear_rate;
    float degree_1 = 0.0f, degree_2 = 0.0f, degree_3 = 0.0f, degree_4 = 0.0f;
    float torque_1 = 0.0f, torque_2 = 0.0f, torque_3 = 0.0f, torque_4 = 0.0f;
    float time = 0.0;
    int stand_f,stand_b; // 支持脚判定変数

    float torque_limit = 0.85f;
    float degree_limit_up = radians(10.0f); //遊脚
    float degree_limit_down = radians(20.0f); //支持脚

    boolean mainmode_flag = false;

    display_clear();
    display_println("main");

    const int BUFFER_SIZE = 10;
    int bufferCount = 0;     // バッファ内データ件数

    LogData logData[BUFFER_SIZE] = {0};

    if (!SD.begin(4)) {
        Serial.println("SDカードの初期化に失敗しました");
        while (M5.BtnB.isPressed() != true) M5.update();
    }

    String filename = makeUniqueFilename("/pace_VPB10.csv");
    logFile = SD.open(filename, FILE_WRITE);
    if (!logFile) {
        Serial.println("ファイルオープン失敗");
        while (M5.BtnB.isPressed() != true) M5.update();
    }

    // ヘッダー
    logFile.println("time_ms,m1_deg,m1_cur,m1_torq,m2_deg,m2_cur,m2_torq,m3_deg,m3_cur,m3_torq,m4_deg,m4_cur,m4_torq,front,back");
    logFile.flush();

    // ドライバーリセット
    for(int i = 1; i <= 4; i++){
        tx_message.identifier = 0x100 + i;
        tx_message.data[0] = 'R';
        tx_message.data_length_code = 1;
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
        int count = 0;
        boolean mainmode_flag = false;
        do{
            delay(1);
            count++;
            if (count > 8000) { // タイムアウト処理
                Serial.print("timeout");
                mainmode_flag = true;
            }
            if(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
                if(rx_message.identifier == 0x200 + i && rx_message.data[0] == 'R'){
                    mainmode_flag = true;
                }
            }
        }while(mainmode_flag == false);
        Serial.printf("%#x:reset\n",0x100+i);
    }

    // キャリブレーション
    for(int i = 1; i <= 4; i++){
        tx_message.identifier = 0x100 + i;
        tx_message.data[0] = 'L';
        tx_message.data_length_code = 1;
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
        int count = 0;
        mainmode_flag = false;
        do{
            delay(1);
            count++;
            if (count > 5000) { // タイムアウト処理
                Serial.print("timeout");
                mainmode_flag = true;
            }
            if(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
                if(rx_message.identifier == 0x200 + i && rx_message.data[0] == 'L'){
                    mainmode_flag = true;
                }
            }
        }while(mainmode_flag == false);
        Serial.printf("%#x:calib\n",0x100+i);
    }

    // 制限設定
    for(int i = 1; i <= 4; i++){
        tx_message.identifier = 0x100 + i;
        tx_message.data[0] = 'B';
        tx_message.data[1] = 'T';
        ((float*)&tx_message.data[2])[0] = 0.8;
        tx_message.data_length_code = 2 + sizeof(float);
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
    }

    unsigned long lastTime = millis();
    int count = 0;
    while(count < 50){    //一定角度で安定するまで待機
        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0f;
        lastTime = now;
        lsm1.getEvent(&a1, &mag1, &g1, &temp1);
        lsm2.getEvent(&a2, &mag2, &g2, &temp2);
        pitchAcc1 = atan2(-a1.acceleration.x, sqrt(a1.acceleration.y * a1.acceleration.y + a1.acceleration.z * a1.acceleration.z)) * RAD_TO_DEG;
        pitchAcc2 = atan2(-a2.acceleration.x, sqrt(a2.acceleration.y * a2.acceleration.y + a2.acceleration.z * a2.acceleration.z)) * RAD_TO_DEG;
        pitch1 = kalmanPitch1.update(pitchAcc1, -g1.gyro.y * RAD_TO_DEG, dt);
        pitch2 = kalmanPitch2.update(pitchAcc2, -g2.gyro.y * RAD_TO_DEG, dt);
        if(pitch1 <= front_deg + front_deg_range && pitch1 >= front_deg - front_deg_range && pitch2 <= -front_deg + front_deg_range && pitch2 >= -front_deg - front_deg_range){
            count++;
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.setTextColor(GREEN,BLACK);
            canvas.drawCenterString(String(pitch1,3), M5.Display.width() / 2, M5.Display.height() / 2 -20);
            canvas.drawCenterString(String(pitch2,3), M5.Display.width() / 2, M5.Display.height() / 2 +20);
            canvas.pushSprite(0, 0);
        }else{
            count = 0;
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.drawCenterString(String(pitch1,3), M5.Display.width() / 2, M5.Display.height() / 2 -20);
            canvas.drawCenterString(String(pitch2,3), M5.Display.width() / 2, M5.Display.height() / 2 +20);
            canvas.pushSprite(0, 0);
        }
    }
    Serial.println("degree ready");
// */
    while(pitch1 >= 0){    //0°を跨ぐまで待機
        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0f;
        lastTime = now;
        lsm1.getEvent(&a1, &mag1, &g1, &temp1);
        lsm2.getEvent(&a2, &mag2, &g2, &temp2);
        pitchAcc1 = atan2(-a1.acceleration.x, sqrt(a1.acceleration.y * a1.acceleration.y + a1.acceleration.z * a1.acceleration.z)) * RAD_TO_DEG;
        pitchAcc2 = atan2(-a2.acceleration.x, sqrt(a2.acceleration.y * a2.acceleration.y + a2.acceleration.z * a2.acceleration.z)) * RAD_TO_DEG;
        pitch1 = kalmanPitch1.update(pitchAcc1, -g1.gyro.y * RAD_TO_DEG, dt);
        pitch2 = kalmanPitch2.update(pitchAcc2, -g2.gyro.y * RAD_TO_DEG, dt);
        if(pitch1 <= front_deg + front_deg_range && pitch1 >= front_deg - front_deg_range && pitch2 <= -front_deg + front_deg_range && pitch2 >= -front_deg - front_deg_range){
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.setCursor(0,20);
            canvas.print("degree ready");
            canvas.setTextColor(GREEN,BLACK);
            canvas.drawCenterString(String(pitch1,3), M5.Display.width() / 2, M5.Display.height() / 2 -20);
            canvas.drawCenterString(String(pitch2,3), M5.Display.width() / 2, M5.Display.height() / 2 +20);
            canvas.pushSprite(0, 0);
        }else{
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.setCursor(0,20);
            canvas.print("degree ready");
            canvas.drawCenterString(String(pitch1,3), M5.Display.width() / 2, M5.Display.height() / 2 -20);
            canvas.drawCenterString(String(pitch2,3), M5.Display.width() / 2, M5.Display.height() / 2 +20);
            canvas.pushSprite(0, 0);
        }
    }
    Serial.println("GO");

    count = 0;

    SD_setup(BUFFER_SIZE);

    lsm1.getEvent(&a1, &mag1, &g1, &temp1);
    boolean g1_sign;
    if(g1.gyro.y >= 0.0) g1_sign = true;
    else g1_sign = false;

    // 割り込み設定
    // Initialize intrupt
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, control_cycle, true);
    timerAlarmEnable(timer);

    while(count < 2){
        while(Loop_flag == 0);
        Loop_flag = 0;

        lsm1.getEvent(&a1, &mag1, &g1, &temp1);
        lsm2.getEvent(&a2, &mag2, &g2, &temp2);
        pitchAcc1 = atan2(-a1.acceleration.x, sqrt(a1.acceleration.y * a1.acceleration.y + a1.acceleration.z * a1.acceleration.z)) * RAD_TO_DEG;
        pitchAcc2 = atan2(-a2.acceleration.x, sqrt(a2.acceleration.y * a2.acceleration.y + a2.acceleration.z * a2.acceleration.z)) * RAD_TO_DEG;
        pitch1 = kalmanPitch1.update(pitchAcc1, -g1.gyro.y * RAD_TO_DEG, 0.005);
        pitch2 = kalmanPitch2.update(pitchAcc2, -g2.gyro.y * RAD_TO_DEG, 0.005);

        front_fleq = PLL(pitch1, &front_omega, &front_phi, 5);
        back_fleq = PLL(pitch2, &back_omega, &back_phi, 5);

        if(g1_sign == true && g1.gyro.y < 0.0){
            count++;
            g1_sign = false;
        }else if(g1_sign == false && g1.gyro.y >= 0.0){
            count++;
            g1_sign = true;
        }
    }

    count = 0;

    while(count < 3){
        while(Loop_flag == 0);
        Loop_flag = 0;

        lsm1.getEvent(&a1, &mag1, &g1, &temp1);
        lsm2.getEvent(&a2, &mag2, &g2, &temp2);
        pitchAcc1 = atan2(-a1.acceleration.x, sqrt(a1.acceleration.y * a1.acceleration.y + a1.acceleration.z * a1.acceleration.z)) * RAD_TO_DEG;
        pitchAcc2 = atan2(-a2.acceleration.x, sqrt(a2.acceleration.y * a2.acceleration.y + a2.acceleration.z * a2.acceleration.z)) * RAD_TO_DEG;
        pitch1 = kalmanPitch1.update(pitchAcc1, -g1.gyro.y * RAD_TO_DEG, 0.005);
        pitch2 = kalmanPitch2.update(pitchAcc2, -g2.gyro.y * RAD_TO_DEG, 0.005);

        front_fleq = PLL(pitch1, &front_omega, &front_phi, 5);
        back_fleq = PLL(pitch2, &back_omega, &back_phi, 5);

        degree_1 = -max_degree * cos(front_phi);
        degree_2 = -max_degree * cos(front_phi);
        degree_3 = -max_degree * cos(back_phi);
        degree_4 = -max_degree * cos(back_phi);

        tx_message.identifier = 0x101;
        tx_message.data[0] = 'P';
        ((float*)&tx_message.data[1])[0] = degree_1;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));

        tx_message.identifier = 0x102;
        tx_message.data[0] = 'P';
        ((float*)&tx_message.data[1])[0] = degree_2;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));

        tx_message.identifier = 0x103;
        tx_message.data[0] = 'P';
        ((float*)&tx_message.data[1])[0] = degree_3;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));

        tx_message.identifier = 0x104;
        tx_message.data[0] = 'P';
        ((float*)&tx_message.data[1])[0] = degree_4;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));

        while(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
            if(rx_message.identifier == 0x201){
                logData[bufferCount].m1_deg = *((float*)&rx_message.data[0]);
                logData[bufferCount].m1_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x202){
                logData[bufferCount].m2_deg = *((float*)&rx_message.data[0]);
                logData[bufferCount].m2_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x203){
                logData[bufferCount].m3_deg = *((float*)&rx_message.data[0]);
                logData[bufferCount].m3_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x204){
                logData[bufferCount].m4_deg = *((float*)&rx_message.data[0]);
                logData[bufferCount].m4_cur = *((float*)&rx_message.data[4]);
            }
        }
        logData[bufferCount].m1_torq = degree_1;
        logData[bufferCount].m2_torq = front_phi;
        logData[bufferCount].m3_torq = degree_3;
        logData[bufferCount].m4_torq = back_phi;

        logData[bufferCount].time_ms = millis();
        logData[bufferCount].front = radians(pitch1);
        logData[bufferCount].back = radians(pitch2);
        
        bufferCount++;

        if (bufferCount >= BUFFER_SIZE) {
            xQueueSend(sdQueue, logData, portMAX_DELAY);
            bufferCount = 0;
        }

        if(g1_sign == true && g1.gyro.y < 0.0){
            count++;
            g1_sign = false;
        }else if(g1_sign == false && g1.gyro.y >= 0.0){
            count++;
            g1_sign = true;
        }
    }

    while(true){
        // 割り込みにより200Hzで以降のコードが実行
        while (Loop_flag == 0);
        Loop_flag = 0;
        M5.update();
        if(M5.BtnA.isPressed()){
            break;
        }

        //kalman_update_2(&roll1,&pitch1,&yaw1,&roll2,&pitch2,&yaw2,0.005);
        lsm1.getEvent(&a1, &mag1, &g1, &temp1);
        lsm2.getEvent(&a2, &mag2, &g2, &temp2);
        pitchAcc1 = atan2(-a1.acceleration.x, sqrt(a1.acceleration.y * a1.acceleration.y + a1.acceleration.z * a1.acceleration.z)) * RAD_TO_DEG;
        pitchAcc2 = atan2(-a2.acceleration.x, sqrt(a2.acceleration.y * a2.acceleration.y + a2.acceleration.z * a2.acceleration.z)) * RAD_TO_DEG;
        pitch1 = kalmanPitch1.update(pitchAcc1, -g1.gyro.y * RAD_TO_DEG, 0.005);
        pitch2 = kalmanPitch2.update(pitchAcc2, -g2.gyro.y * RAD_TO_DEG, 0.005);

        front_fleq = PLL(pitch1, &front_omega, &front_phi, 5);
        back_fleq = PLL(pitch2, &back_omega, &back_phi, 5);

        time += control_cycle/1000;

        stand_f = stand_judge(-radians(pitch1), degree_1, -degree_2);
        stand_b = stand_judge(radians(pitch2), degree_3, -degree_4);

        tan_phi = abs(tan(speed * PI / 2.0 / front_omega));

        if(stand_f == 1){   // 右(m1)が支持脚
            torque_1 = -u1 * cos(degree_1) * tan_phi;
            torque_2 = -u2 * cos(degree_2) * tan_phi;
            // canvas.setTextColor(WHITE,BLACK);
        }else if(stand_f == 2){ // 左(m2)が支持脚
            torque_1 = u2 * cos(degree_1) * tan_phi;
            torque_2 = u1 * cos(degree_2) * tan_phi;
            // canvas.setTextColor(GREEN,BLACK);
        }else{
            torque_1 = 0.0;
            torque_2 = 0.0;
            // canvas.setTextColor(RED,BLACK);
        }
        // canvas.drawCenterString(String(pitch1,3), M5.Display.width() / 2, M5.Display.height() / 2 -20);
        if(stand_b == 1){ // 右(m3)が支持脚
            torque_3 = -u1 * cos(degree_3) * tan_phi;
            torque_4 = -u2 * cos(degree_4) * 2.0 * tan_phi;
            // canvas.setTextColor(GREEN,BLACK);
        }else if(stand_b == 2){ // 左(m4)が支持脚
            torque_3 = u2 * cos(degree_3) * tan_phi;
            torque_4 = u1 * cos(degree_4) * 2.0 * tan_phi;
            // canvas.setTextColor(WHITE,BLACK);
        }else{
            torque_3 = 0.0;
            torque_4 = 0.0;
            // canvas.setTextColor(RED,BLACK);
        }
        // canvas.drawCenterString(String(pitch2,3), M5.Display.width() / 2, M5.Display.height() / 2 +20);
        // canvas.pushSprite(0, 0);

        if(isfinite(torque_1) == false) torque_1 = 0.0;
        if(isfinite(torque_2) == false) torque_2 = 0.0;
        if(isfinite(torque_3) == false) torque_3 = 0.0;
        if(isfinite(torque_4) == false) torque_4 = 0.0;

        tx_message.identifier = 0x101;
        if(degree_1 >= degree_limit_up-radians(0.5) && stand_f == 2) { //遊脚
            tx_message.data[0] = 'P';
            ((float*)&tx_message.data[1])[0] = degree_limit_up;
        }else if(degree_1 <= -degree_limit_down+radians(0.5) && stand_f == 1) { //支持脚
            tx_message.data[0] = 'P';
            ((float*)&tx_message.data[1])[0] = -degree_limit_down;
        }else{
            tx_message.data[0] = 'T';
            ((float*)&tx_message.data[1])[0] = torque_1;
        }
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));
        //ets_delay_us(200);  // 200µs

        tx_message.identifier = 0x102;
        if(degree_2 >= degree_limit_down-radians(0.5) && stand_f == 2) { //支持脚
            tx_message.data[0] = 'P';
            ((float*)&tx_message.data[1])[0] = degree_limit_down;
        }else if(degree_2 <= -degree_limit_up+radians(0.5) && stand_f == 1) {
            tx_message.data[0] = 'P';
            ((float*)&tx_message.data[1])[0] = -degree_limit_up;
        }else{
            tx_message.data[0] = 'T';
            ((float*)&tx_message.data[1])[0] = torque_2;
        }
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));
        //ets_delay_us(200);  // 200µs

        tx_message.identifier = 0x103;
        if(degree_3 >= degree_limit_up-radians(0.5) && stand_b == 2) { //遊脚
            tx_message.data[0] = 'P';
            ((float*)&tx_message.data[1])[0] = degree_limit_up;
        }else if(degree_3 <= -degree_limit_down+radians(0.5) && stand_b == 1) { //支持脚
            tx_message.data[0] = 'P';
            ((float*)&tx_message.data[1])[0] = -degree_limit_down;
        }else{
            tx_message.data[0] = 'T';
            ((float*)&tx_message.data[1])[0] = torque_3;
        }
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));
        //ets_delay_us(200);  // 200µs

        tx_message.identifier = 0x104;
        if(degree_4 >= degree_limit_down-radians(0.5) && stand_b == 2) { //支持脚
            tx_message.data[0] = 'P';
            ((float*)&tx_message.data[1])[0] = degree_limit_down;
        }else if(degree_4 <= -degree_limit_up+radians(0.5) && stand_b == 1) {
            tx_message.data[0] = 'P';
            ((float*)&tx_message.data[1])[0] = -degree_limit_up;
        }else{
            tx_message.data[0] = 'T';
            ((float*)&tx_message.data[1])[0] = torque_4;
        }
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));
        //ets_delay_us(200);  // 200µs

        logData[bufferCount] = {0};

        logData[bufferCount].m1_torq = torque_1;
        logData[bufferCount].m2_torq = torque_2;
        logData[bufferCount].m3_torq = torque_3;
        logData[bufferCount].m4_torq = torque_4;

        while(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
            if(rx_message.identifier == 0x201){
                logData[bufferCount].m1_deg = *((float*)&rx_message.data[0]);
                degree_1 = logData[bufferCount].m1_deg;
                logData[bufferCount].m1_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x202){
                logData[bufferCount].m2_deg = *((float*)&rx_message.data[0]);
                degree_2 = logData[bufferCount].m2_deg;
                logData[bufferCount].m2_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x203){
                logData[bufferCount].m3_deg = *((float*)&rx_message.data[0]);
                degree_3 = logData[bufferCount].m3_deg;
                logData[bufferCount].m3_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x204){
                logData[bufferCount].m4_deg = *((float*)&rx_message.data[0]);
                degree_4 = logData[bufferCount].m4_deg;
                logData[bufferCount].m4_cur = *((float*)&rx_message.data[4]);
            }
        }

        logData[bufferCount].time_ms = millis();
        logData[bufferCount].front = radians(pitch1);
        logData[bufferCount].back = radians(pitch2);

        bufferCount++;

        if (bufferCount >= BUFFER_SIZE) {
            xQueueSend(sdQueue, logData, portMAX_DELAY);
            bufferCount = 0;
        }
    }
    timerAlarmDisable(timer);

    xQueueSend(sdQueue, logData, portMAX_DELAY);

    tx_message.identifier = 0x101;
    tx_message.data[0] = 'E';
    tx_message.data[1] = '0';
    tx_message.data_length_code = 2;
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    tx_message.identifier = 0x102;
    tx_message.data[0] = 'E';
    tx_message.data[1] = '0';
    tx_message.data_length_code = 2;
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    tx_message.identifier = 0x103;
    tx_message.data[0] = 'E';
    tx_message.data[1] = '0';
    tx_message.data_length_code = 2;
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    tx_message.identifier = 0x104;
    tx_message.data[0] = 'E';
    tx_message.data[1] = '0';
    tx_message.data_length_code = 2;
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    delay(1000);
    logFile.close();

    ESP.restart();
}

void pace_VirtualPassiveBase2_rev(void){
    const float front_deg = 20;
    const float front_deg_range = 3;

    float roll,pitch,yaw;

    const float tan_phi = tan(radians(5)); //仮想重力の傾きtanφ

    const int control_cycle = 5000; //制御周期(us)
    const int gear_rate = 16;
    const float KT = 0.150; //トルク定数(Nm/A)
    const float L1 = 0.3;
    const float m1 = 0.564;
    const float m2 = 0.378;
    const float m3 = 0.735;
    const float g = 9.81;
    const float u1 = 0.3f * L1 * (3*m1 + 4*m2 + m3) * g * tan_phi / KT / gear_rate;
    const float u2 = 2.5f * m1 * L1 * g * tan_phi / KT / gear_rate;
    float degree_1 = 0.0f, degree_2 = 0.0f, degree_3 = 0.0f, degree_4 = 0.0f;
    float torque_1 = 0.0f, torque_2 = 0.0f, torque_3 = 0.0f, torque_4 = 0.0f;
    float time = 0.0;

    float torque_limit = 0.85f;
    float degree_limit = 15.0f;

    boolean mainmode_flag = false;

    display_clear();
    display_println("main");

    const int BUFFER_SIZE = 10;
    int bufferCount = 0;     // バッファ内データ件数

    LogData logData[BUFFER_SIZE] = {0};

    if (!SD.begin(4)) {
        Serial.println("SDカードの初期化に失敗しました");
        while (1);
    }

    String filename = makeUniqueFilename("/pace_VPB_rev.csv");
    logFile = SD.open(filename, FILE_WRITE);
    if (!logFile) {
        Serial.println("ファイルオープン失敗");
        while (1);
    }

    // ヘッダー
    logFile.println("time_ms,m1_deg,m1_cur,m1_torq,m2_deg,m2_cur,m2_torq,m3_deg,m3_cur,m3_torq,m4_deg,m4_cur,m4_torq,front,back");
    logFile.flush();

    // ドライバーリセット
    for(int i = 1; i <= 4; i++){
        tx_message.identifier = 0x100 + i;
        tx_message.data[0] = 'R';
        tx_message.data_length_code = 1;
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
        int count = 0;
        boolean mainmode_flag = false;
        do{
            delay(1);
            count++;
            if (count > 8000) { // タイムアウト処理
                Serial.print("timeout");
                mainmode_flag = true;
            }
            if(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
                if(rx_message.identifier == 0x200 + i && rx_message.data[0] == 'R'){
                    mainmode_flag = true;
                }
            }
        }while(mainmode_flag == false);
        Serial.printf("%#x:reset\n",0x100+i);
    }

    // キャリブレーション
    for(int i = 1; i <= 4; i++){
        tx_message.identifier = 0x100 + i;
        tx_message.data[0] = 'L';
        tx_message.data_length_code = 1;
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
        int count = 0;
        mainmode_flag = false;
        do{
            delay(1);
            count++;
            if (count > 5000) { // タイムアウト処理
                Serial.print("timeout");
                mainmode_flag = true;
            }
            if(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
                if(rx_message.identifier == 0x200 + i && rx_message.data[0] == 'L'){
                    mainmode_flag = true;
                }
            }
        }while(mainmode_flag == false);
        Serial.printf("%#x:calib\n",0x100+i);
    }

    // 制限設定
    for(int i = 1; i <= 4; i++){
        tx_message.identifier = 0x100 + i;
        tx_message.data[0] = 'B';
        tx_message.data[1] = 'A';
        ((float*)&tx_message.data[2])[0] = degree_limit;
        tx_message.data_length_code = 2 + sizeof(float);
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
    }

    int count = 0;
    while(count < 50){    //一定角度で安定するまで待機
        kalman_update(&roll,&pitch,&yaw);
        if(pitch <= -front_deg + front_deg_range && pitch >= -front_deg - front_deg_range){
            count++;
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.setTextColor(GREEN,BLACK);
            canvas.drawCenterString(String(pitch,3), M5.Display.width() / 2, M5.Display.height() / 2);
            canvas.pushSprite(0, 0);
        }else{
            count = 0;
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.drawCenterString(String(pitch,3), M5.Display.width() / 2, M5.Display.height() / 2);
            canvas.pushSprite(0, 0);
        }
    }
    Serial.println("degree ready");
// */
    while(pitch <= 0){    //0°を跨ぐまで待機
        kalman_update(&roll,&pitch,&yaw);
        if(pitch <= -front_deg + front_deg_range && pitch >= -front_deg - front_deg_range){
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.setCursor(0,20);
            canvas.print("degree ready");
            canvas.setTextColor(GREEN,BLACK);
            canvas.drawCenterString(String(pitch,3), M5.Display.width() / 2, M5.Display.height() / 2);
            canvas.pushSprite(0, 0);
        }else{
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.setCursor(0,20);
            canvas.print("degree ready");
            canvas.drawCenterString(String(pitch,3), M5.Display.width() / 2, M5.Display.height() / 2);
            canvas.pushSprite(0, 0);
        }
    }
    Serial.println("GO");

    // float gyroX, gyroY, gyroZ;
    // M5.update();
    // M5.Imu.getGyroData(&gyroX, &gyroY, &gyroZ);
    // if(gyroY > 0){
    //     while(gyroY >= 0){
    //         M5.update();
    //         kalman_update(&roll,&pitch,&yaw);
    //         M5.Imu.getGyroData(&gyroX, &gyroY, &gyroZ);
    //         //sine_fleq = estimatePeriod(pitch, gyroY);
    //     }
    // }else{
    //     while(gyroY <= 0){
    //         M5.update();
    //         kalman_update(&roll,&pitch,&yaw);
    //         M5.Imu.getGyroData(&gyroX, &gyroY, &gyroZ);
    //         //sine_fleq = estimatePeriod(pitch, gyroY);
    //     }
    // }

    for(int i = 1; i <= 4; i++){
        tx_message.identifier = 0x100 + i;
        tx_message.data[0] = 'M';
        tx_message.data[1] = 'T';
        ((float*)&tx_message.data[2])[0] = 0.0;
        tx_message.data_length_code = 2 + sizeof(float); // 6
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
    }
    
    count = 0;

    SD_setup(BUFFER_SIZE);

    // 割り込み設定
    // Initialize intrupt
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, control_cycle, true);
    timerAlarmEnable(timer);

    while(true){
        // 割り込みにより200Hzで以降のコードが実行
        while (Loop_flag == 0);
        Loop_flag = 0;
        M5.update();
        if(M5.BtnA.isPressed()){
            break;
        }

        kalman_update(&roll,&pitch,&yaw);
        //M5.Imu.getGyroData(&gyroX, &gyroY, &gyroZ);
        //sine_fleq = estimatePeriod(pitch, gyroY);

        time += control_cycle/1000;

        if(pitch > 1.0){   //機体がどちらに傾いているか（どちらの脚をついているか）
            torque_1 = -u1 * cos(degree_1) * abs(degree_3 / (abs(degree_1) + abs(degree_3)));
            torque_2 = -u2 * cos(degree_2) * abs(degree_4 / (abs(degree_2) + abs(degree_4)));
            torque_3 = -u1 * cos(degree_3) * abs(degree_1 / (abs(degree_3) + abs(degree_1)));
            torque_4 = -u2 * cos(degree_4) * abs(degree_2 / (abs(degree_2) + abs(degree_4)));
        }else if(pitch < -1.0){
            torque_1 = u2 * cos(degree_1) * abs(degree_3 / (abs(degree_1) + abs(degree_3)));
            torque_2 = u1 * cos(degree_2) * abs(degree_4 / (abs(degree_2) + abs(degree_4)));
            torque_3 = u2 * cos(degree_3) * abs(degree_1 / (abs(degree_3) + abs(degree_1)));
            torque_4 = u1 * cos(degree_4) * abs(degree_2 / (abs(degree_4) + abs(degree_2)));
        }else{
            torque_1 = 0.0;
            torque_2 = 0.0;
            torque_3 = 0.0;
            torque_4 = 0.0;
        }

        if(isfinite(torque_1) == false) torque_1 = 0.0;
        if(isfinite(torque_2) == false) torque_2 = 0.0;
        if(isfinite(torque_3) == false) torque_3 = 0.0;
        if(isfinite(torque_4) == false) torque_4 = 0.0;

        tx_message.identifier = 0x101;
        tx_message.data[0] = 'T';
        ((float*)&tx_message.data[1])[0] = torque_1;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));
        //ets_delay_us(200);  // 200µs

        tx_message.identifier = 0x102;
        tx_message.data[0] = 'T';
        ((float*)&tx_message.data[1])[0] = torque_2;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));
        //ets_delay_us(200);  // 200µs

        tx_message.identifier = 0x103;
        tx_message.data[0] = 'T';
        ((float*)&tx_message.data[1])[0] = torque_3;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));
        //ets_delay_us(200);  // 200µs

        tx_message.identifier = 0x104;
        tx_message.data[0] = 'T';
        ((float*)&tx_message.data[1])[0] = torque_4;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));
        //ets_delay_us(200);  // 200µs

        logData[bufferCount] = {0};

        logData[bufferCount].m1_torq = torque_1;
        logData[bufferCount].m2_torq = torque_2;
        logData[bufferCount].m3_torq = torque_3;
        logData[bufferCount].m4_torq = torque_4;

        while(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
            if(rx_message.identifier == 0x201){
                logData[bufferCount].m1_deg = *((float*)&rx_message.data[0]);
                degree_1 = logData[bufferCount].m1_deg;
                logData[bufferCount].m1_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x202){
                logData[bufferCount].m2_deg = *((float*)&rx_message.data[0]);
                degree_2 = logData[bufferCount].m2_deg;
                logData[bufferCount].m2_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x203){
                logData[bufferCount].m3_deg = *((float*)&rx_message.data[0]);
                degree_3 = logData[bufferCount].m3_deg;
                logData[bufferCount].m3_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x204){
                logData[bufferCount].m4_deg = *((float*)&rx_message.data[0]);
                degree_4 = logData[bufferCount].m4_deg;
                logData[bufferCount].m4_cur = *((float*)&rx_message.data[4]);
            }
        }

        logData[bufferCount].time_ms = millis();
        logData[bufferCount].front = radians(pitch);

        bufferCount++;

        if (bufferCount >= BUFFER_SIZE) {
            xQueueSend(sdQueue, logData, portMAX_DELAY);
            bufferCount = 0;
        }

    }
    timerAlarmDisable(timer);

    xQueueSend(sdQueue, logData, portMAX_DELAY);

    tx_message.identifier = 0x101;
    tx_message.data[0] = 'E';
    tx_message.data[1] = '0';
    tx_message.data_length_code = 2;
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    tx_message.identifier = 0x102;
    tx_message.data[0] = 'E';
    tx_message.data[1] = '0';
    tx_message.data_length_code = 2;
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    tx_message.identifier = 0x103;
    tx_message.data[0] = 'E';
    tx_message.data[1] = '0';
    tx_message.data_length_code = 2;
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    tx_message.identifier = 0x104;
    tx_message.data[0] = 'E';
    tx_message.data[1] = '0';
    tx_message.data_length_code = 2;
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    delay(1000);
    logFile.close();

    ESP.restart();
}

void pace_FixCycle2(void){
    const float front_deg = 30;
    const float front_deg_range = 3;

    float roll1,pitch1,yaw1;
    float roll2,pitch2,yaw2;
    SimpleKalmanFilter kalmanPitch1;
    SimpleKalmanFilter kalmanPitch2;
    sensors_event_t a1, mag1, g1, temp1;
    sensors_event_t a2, mag2, g2, temp2; 
    float pitchAcc1, pitchAcc2;

    float front_fleq,back_fleq;
    float front_omega = 0.0, back_omega = 0.0;
    float front_phi = 0.0, back_phi = 0.0;
    int front_lastcross, back_lastcross;

    const int control_cycle = 5000; //制御周期(us)
    const int gear_rate = 16;
    const float max_degree = radians(7.8);
    const float degree_delta = radians(3.0);
    float sine_fleq = 0.8;//0.679; //sine波周波数(Hz)

    float degree_1,degree_2,degree_3,degree_4;
    float old_degree = 0.0;
    float time = 0.0;

    boolean mainmode_flag = false;

    display_clear();
    display_println("main");

    const int BUFFER_SIZE = 10;
    // String dataBuffer = "";  // 書き込み用バッファ
    int bufferCount = 0;     // バッファ内データ件数
    // float m1deg,m2deg,m3deg,m4deg = 0.0;
    // float m1cur,m2cur,m3cur,m4cur = 0.0;

    LogData logData[BUFFER_SIZE] = {0};

    if (!SD.begin(4)) {
        Serial.println("SDカードの初期化に失敗しました");
        while (1);
    }

    String filename = makeUniqueFilename("/pace_fixCycle.csv");
    logFile = SD.open(filename, FILE_WRITE);
    if (!logFile) {
        Serial.println("ファイルオープン失敗");
        while (1);
    }

    // ヘッダー
    logFile.println("time_ms,m1_deg,m1_cur,,m2_deg,m2_cur,front_phi,m3_deg,m3_cur,,m4_deg,m4_cur,back_phi,front,back");
    logFile.flush();

    // ドライバーリセット
    for(int i = 1; i <= 4; i++){
        tx_message.identifier = 0x100 + i;
        tx_message.data[0] = 'R';
        tx_message.data_length_code = 1;
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
        int count = 0;
        boolean mainmode_flag = false;
        do{
            delay(1);
            count++;
            if (count > 8000) { // タイムアウト処理
                Serial.print("timeout");
                mainmode_flag = true;
            }
            if(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
                if(rx_message.identifier == 0x200 + i && rx_message.data[0] == 'R'){
                    mainmode_flag = true;
                }
            }
        }while(mainmode_flag == false);
        Serial.printf("%#x:reset\n",0x100+i);
    }

    // キャリブレーション
    for(int i = 1; i <= 4; i++){
        tx_message.identifier = 0x100 + i;
        tx_message.data[0] = 'L';
        tx_message.data_length_code = 1;
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
        int count = 0;
        mainmode_flag = false;
        do{
            delay(1);
            count++;
            if (count > 5000) { // タイムアウト処理
                Serial.print("timeout");
                mainmode_flag = true;
            }
            if(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
                if(rx_message.identifier == 0x200 + i && rx_message.data[0] == 'L'){
                    mainmode_flag = true;
                }
            }
        }while(mainmode_flag == false);
        Serial.printf("%#x:calib\n",0x100+i);
    }

    unsigned long lastTime = millis();
    int count = 0;
    while(count < 50){    //一定角度で安定するまで待機
        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0f;
        lastTime = now;
        lsm1.getEvent(&a1, &mag1, &g1, &temp1);
        lsm2.getEvent(&a2, &mag2, &g2, &temp2);
        pitchAcc1 = atan2(-a1.acceleration.x, sqrt(a1.acceleration.y * a1.acceleration.y + a1.acceleration.z * a1.acceleration.z)) * RAD_TO_DEG;
        pitchAcc2 = atan2(-a2.acceleration.x, sqrt(a2.acceleration.y * a2.acceleration.y + a2.acceleration.z * a2.acceleration.z)) * RAD_TO_DEG;
        pitch1 = kalmanPitch1.update(pitchAcc1, -g1.gyro.y * RAD_TO_DEG, dt);
        pitch2 = kalmanPitch2.update(pitchAcc2, -g2.gyro.y * RAD_TO_DEG, dt);
        if(pitch1 <= front_deg + front_deg_range && pitch1 >= front_deg - front_deg_range && pitch2 <= -front_deg + front_deg_range && pitch2 >= -front_deg - front_deg_range){
            count++;
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.setTextColor(GREEN,BLACK);
            canvas.drawCenterString(String(pitch1,3), M5.Display.width() / 2, M5.Display.height() / 2 -20);
            canvas.drawCenterString(String(pitch2,3), M5.Display.width() / 2, M5.Display.height() / 2 +20);
            canvas.pushSprite(0, 0);
        }else{
            count = 0;
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.drawCenterString(String(pitch1,3), M5.Display.width() / 2, M5.Display.height() / 2 -20);
            canvas.drawCenterString(String(pitch2,3), M5.Display.width() / 2, M5.Display.height() / 2 +20);
            canvas.pushSprite(0, 0);
        }
    }
    Serial.println("degree ready");
// */
    while(pitch1 >= 0){    //0°を跨ぐまで待機
        unsigned long now = millis();
        float dt = now - lastTime;
        lastTime = now;
        lsm1.getEvent(&a1, &mag1, &g1, &temp1);
        lsm2.getEvent(&a2, &mag2, &g2, &temp2);
        pitchAcc1 = atan2(-a1.acceleration.x, sqrt(a1.acceleration.y * a1.acceleration.y + a1.acceleration.z * a1.acceleration.z)) * RAD_TO_DEG;
        pitchAcc2 = atan2(-a2.acceleration.x, sqrt(a2.acceleration.y * a2.acceleration.y + a2.acceleration.z * a2.acceleration.z)) * RAD_TO_DEG;
        pitch1 = kalmanPitch1.update(pitchAcc1, -g1.gyro.y * RAD_TO_DEG, dt/1000.0);
        pitch2 = kalmanPitch2.update(pitchAcc2, -g2.gyro.y * RAD_TO_DEG, dt/1000.0);

        front_fleq = PLL(pitch1, &front_omega, &front_phi, dt);
        back_fleq = PLL(pitch2, &back_omega, &back_phi, dt);
        if(pitch1 <= front_deg + front_deg_range && pitch1 >= front_deg - front_deg_range && pitch2 <= -front_deg + front_deg_range && pitch2 >= -front_deg - front_deg_range){
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.setCursor(0,20);
            canvas.print("degree ready");
            canvas.setTextColor(GREEN,BLACK);
            canvas.drawCenterString(String(pitch1,3), M5.Display.width() / 2, M5.Display.height() / 2 -20);
            canvas.drawCenterString(String(pitch2,3), M5.Display.width() / 2, M5.Display.height() / 2 +20);
            canvas.pushSprite(0, 0);
        }else{
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.setCursor(0,20);
            canvas.print("degree ready");
            canvas.drawCenterString(String(pitch1,3), M5.Display.width() / 2, M5.Display.height() / 2 -20);
            canvas.drawCenterString(String(pitch2,3), M5.Display.width() / 2, M5.Display.height() / 2 +20);
            canvas.pushSprite(0, 0);
        }
    }
    Serial.println("GO");
    
    count = 0;

    SD_setup(BUFFER_SIZE);

    lsm1.getEvent(&a1, &mag1, &g1, &temp1);
    boolean g1_sign;
    if(g1.gyro.y >= 0.0) g1_sign = true;
    else g1_sign = false;

    // 割り込み設定
    // Initialize intrupt
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, control_cycle, true);
    timerAlarmEnable(timer);

    while(count < 2){
        while(Loop_flag == 0);
        Loop_flag = 0;

        lsm1.getEvent(&a1, &mag1, &g1, &temp1);
        lsm2.getEvent(&a2, &mag2, &g2, &temp2);
        pitchAcc1 = atan2(-a1.acceleration.x, sqrt(a1.acceleration.y * a1.acceleration.y + a1.acceleration.z * a1.acceleration.z)) * RAD_TO_DEG;
        pitchAcc2 = atan2(-a2.acceleration.x, sqrt(a2.acceleration.y * a2.acceleration.y + a2.acceleration.z * a2.acceleration.z)) * RAD_TO_DEG;
        pitch1 = kalmanPitch1.update(pitchAcc1, -g1.gyro.y * RAD_TO_DEG, 0.005);
        pitch2 = kalmanPitch2.update(pitchAcc2, -g2.gyro.y * RAD_TO_DEG, 0.005);

        front_fleq = PLL(pitch1, &front_omega, &front_phi, 5);
        back_fleq = PLL(pitch2, &back_omega, &back_phi, 5);

        degree_1 = -max_degree * cos(front_phi);
        degree_3 = -max_degree * cos(back_phi);

        logData[bufferCount].m1_torq = degree_1;
        logData[bufferCount].m2_torq = front_phi;
        logData[bufferCount].m3_torq = degree_3;
        logData[bufferCount].m4_torq = back_phi;

        logData[bufferCount].time_ms = millis();
        logData[bufferCount].front = radians(pitch1);
        logData[bufferCount].back = radians(pitch2);
        
        bufferCount++;

        if (bufferCount >= BUFFER_SIZE) {
            xQueueSend(sdQueue, logData, portMAX_DELAY);
            bufferCount = 0;
        }

        if(g1_sign == true && g1.gyro.y < 0.0){
            count++;
            g1_sign = false;
        }else if(g1_sign == false && g1.gyro.y >= 0.0){
            count++;
            g1_sign = true;
        }
    }

    while(true){
        while(Loop_flag == 0);
        Loop_flag = 0;
        M5.update();
        if(M5.BtnA.isPressed()){
            break;
        }

        lsm1.getEvent(&a1, &mag1, &g1, &temp1);
        lsm2.getEvent(&a2, &mag2, &g2, &temp2);
        pitchAcc1 = atan2(-a1.acceleration.x, sqrt(a1.acceleration.y * a1.acceleration.y + a1.acceleration.z * a1.acceleration.z)) * RAD_TO_DEG;
        pitchAcc2 = atan2(-a2.acceleration.x, sqrt(a2.acceleration.y * a2.acceleration.y + a2.acceleration.z * a2.acceleration.z)) * RAD_TO_DEG;
        pitch1 = kalmanPitch1.update(pitchAcc1, -g1.gyro.y * RAD_TO_DEG, 0.005);
        pitch2 = kalmanPitch2.update(pitchAcc2, -g2.gyro.y * RAD_TO_DEG, 0.005);

        front_fleq = PLL(pitch1, &front_omega, &front_phi, 5);
        back_fleq = PLL(pitch2, &back_omega, &back_phi, 5);

        degree_1 = -max_degree * cos(front_phi);
        degree_2 = -max_degree * cos(front_phi);
        degree_3 = -max_degree * cos(back_phi);
        degree_4 = -max_degree * cos(back_phi);

        tx_message.identifier = 0x101;
        tx_message.data[0] = 'P';
        ((float*)&tx_message.data[1])[0] = degree_1;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));

        tx_message.identifier = 0x102;
        tx_message.data[0] = 'P';
        ((float*)&tx_message.data[1])[0] = degree_2;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));

        tx_message.identifier = 0x103;
        tx_message.data[0] = 'P';
        ((float*)&tx_message.data[1])[0] = degree_3;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));

        tx_message.identifier = 0x104;
        tx_message.data[0] = 'P';
        ((float*)&tx_message.data[1])[0] = degree_4;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));

        while(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
            if(rx_message.identifier == 0x201){
                logData[bufferCount].m1_deg = *((float*)&rx_message.data[0]);
                logData[bufferCount].m1_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x202){
                logData[bufferCount].m2_deg = *((float*)&rx_message.data[0]);
                logData[bufferCount].m2_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x203){
                logData[bufferCount].m3_deg = *((float*)&rx_message.data[0]);
                logData[bufferCount].m3_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x204){
                logData[bufferCount].m4_deg = *((float*)&rx_message.data[0]);
                logData[bufferCount].m4_cur = *((float*)&rx_message.data[4]);
            }
        }
        logData[bufferCount].m1_torq = degree_1;
        logData[bufferCount].m2_torq = front_phi;
        logData[bufferCount].m3_torq = degree_3;
        logData[bufferCount].m4_torq = back_phi;

        logData[bufferCount].time_ms = millis();
        logData[bufferCount].front = radians(pitch1);
        logData[bufferCount].back = radians(pitch2);
        
        bufferCount++;

        if (bufferCount >= BUFFER_SIZE) {
            xQueueSend(sdQueue, logData, portMAX_DELAY);
            bufferCount = 0;
        }
    }
    timerAlarmDisable(timer);

    xQueueSend(sdQueue, logData, portMAX_DELAY);

    tx_message.identifier = 0x101;
    tx_message.data[0] = 'T';
    ((float*)&tx_message.data[1])[0] = 0.0;
    tx_message.data_length_code = 1 + sizeof(float); // 5
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    tx_message.identifier = 0x102;
    tx_message.data[0] = 'T';
    ((float*)&tx_message.data[1])[0] = 0.0;
    tx_message.data_length_code = 1 + sizeof(float); // 5
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    tx_message.identifier = 0x103;
    tx_message.data[0] = 'T';
    ((float*)&tx_message.data[1])[0] = 0.0;
    tx_message.data_length_code = 1 + sizeof(float); // 5
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    tx_message.identifier = 0x104;
    tx_message.data[0] = 'T';
    ((float*)&tx_message.data[1])[0] = 0.0;
    tx_message.data_length_code = 1 + sizeof(float); // 5
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    delay(1000);
    logFile.close();

    ESP.restart();
}

void trot_FixCycle(void){
    const float front_deg = 20;
    const float front_deg_range = 3;

    float roll1,pitch1,yaw1;
    float roll2,pitch2,yaw2;
    SimpleKalmanFilter kalmanPitch1;
    SimpleKalmanFilter kalmanPitch2;
    sensors_event_t a1, mag1, g1, temp1;
    sensors_event_t a2, mag2, g2, temp2; 
    float pitchAcc1, pitchAcc2;

    float front_fleq,back_fleq;
    float front_omega = 0.0, back_omega = 0.0;
    float front_phi = 0.0, back_phi = 0.0;
    int front_lastcross, back_lastcross;

    const int control_cycle = 5000; //制御周期(us)
    const int gear_rate = 16;
    const float max_degree = radians(7.8);
    const float degree_delta = radians(3.0);
    float sine_fleq = 0.8;//0.679; //sine波周波数(Hz)

    float degree_1,degree_2,degree_3,degree_4;
    float old_degree = 0.0;
    float time = 0.0;

    boolean mainmode_flag = false;

    display_clear();
    display_println("main");

    const int BUFFER_SIZE = 10;
    // String dataBuffer = "";  // 書き込み用バッファ
    int bufferCount = 0;     // バッファ内データ件数
    // float m1deg,m2deg,m3deg,m4deg = 0.0;
    // float m1cur,m2cur,m3cur,m4cur = 0.0;

    LogData logData[BUFFER_SIZE] = {0};

    if (!SD.begin(4)) {
        Serial.println("SDカードの初期化に失敗しました");
        while (1);
    }

    String filename = makeUniqueFilename("/trot_fixCycle.csv");
    logFile = SD.open(filename, FILE_WRITE);
    if (!logFile) {
        Serial.println("ファイルオープン失敗");
        while (1);
    }

    // ヘッダー
    logFile.println("time_ms,m1_deg,m1_cur,,m2_deg,m2_cur,front_phi,m3_deg,m3_cur,,m4_deg,m4_cur,back_phi,front,back");
    logFile.flush();

    // ドライバーリセット
    for(int i = 1; i <= 4; i++){
        tx_message.identifier = 0x100 + i;
        tx_message.data[0] = 'R';
        tx_message.data_length_code = 1;
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
        int count = 0;
        boolean mainmode_flag = false;
        do{
            delay(1);
            count++;
            if (count > 8000) { // タイムアウト処理
                Serial.print("timeout");
                mainmode_flag = true;
            }
            if(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
                if(rx_message.identifier == 0x200 + i && rx_message.data[0] == 'R'){
                    mainmode_flag = true;
                }
            }
        }while(mainmode_flag == false);
        Serial.printf("%#x:reset\n",0x100+i);
    }

    // キャリブレーション
    for(int i = 1; i <= 4; i++){
        tx_message.identifier = 0x100 + i;
        tx_message.data[0] = 'L';
        tx_message.data_length_code = 1;
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
        int count = 0;
        mainmode_flag = false;
        do{
            delay(1);
            count++;
            if (count > 5000) { // タイムアウト処理
                Serial.print("timeout");
                mainmode_flag = true;
            }
            if(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
                if(rx_message.identifier == 0x200 + i && rx_message.data[0] == 'L'){
                    mainmode_flag = true;
                }
            }
        }while(mainmode_flag == false);
        Serial.printf("%#x:calib\n",0x100+i);
    }

    unsigned long lastTime = millis();
    int count = 0;
    while(count < 50){    //一定角度で安定するまで待機
        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0f;
        lastTime = now;
        lsm1.getEvent(&a1, &mag1, &g1, &temp1);
        lsm2.getEvent(&a2, &mag2, &g2, &temp2);
        pitchAcc1 = atan2(-a1.acceleration.x, sqrt(a1.acceleration.y * a1.acceleration.y + a1.acceleration.z * a1.acceleration.z)) * RAD_TO_DEG;
        pitchAcc2 = atan2(-a2.acceleration.x, sqrt(a2.acceleration.y * a2.acceleration.y + a2.acceleration.z * a2.acceleration.z)) * RAD_TO_DEG;
        pitch1 = kalmanPitch1.update(pitchAcc1, -g1.gyro.y * RAD_TO_DEG, dt);
        pitch2 = kalmanPitch2.update(pitchAcc2, -g2.gyro.y * RAD_TO_DEG, dt);
        if(pitch1 <= front_deg + front_deg_range && pitch1 >= front_deg - front_deg_range && pitch2 <= front_deg + front_deg_range && pitch2 >= front_deg - front_deg_range){
            count++;
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.setTextColor(GREEN,BLACK);
            canvas.drawCenterString(String(pitch1,3), M5.Display.width() / 2, M5.Display.height() / 2 -20);
            canvas.drawCenterString(String(pitch2,3), M5.Display.width() / 2, M5.Display.height() / 2 +20);
            canvas.pushSprite(0, 0);
        }else{
            count = 0;
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.drawCenterString(String(pitch1,3), M5.Display.width() / 2, M5.Display.height() / 2 -20);
            canvas.drawCenterString(String(pitch2,3), M5.Display.width() / 2, M5.Display.height() / 2 +20);
            canvas.pushSprite(0, 0);
        }
    }
    Serial.println("degree ready");
// */
    while(pitch1 >= 0){    //0°を跨ぐまで待機
        unsigned long now = millis();
        float dt = now - lastTime;
        lastTime = now;
        lsm1.getEvent(&a1, &mag1, &g1, &temp1);
        lsm2.getEvent(&a2, &mag2, &g2, &temp2);
        pitchAcc1 = atan2(-a1.acceleration.x, sqrt(a1.acceleration.y * a1.acceleration.y + a1.acceleration.z * a1.acceleration.z)) * RAD_TO_DEG;
        pitchAcc2 = atan2(-a2.acceleration.x, sqrt(a2.acceleration.y * a2.acceleration.y + a2.acceleration.z * a2.acceleration.z)) * RAD_TO_DEG;
        pitch1 = kalmanPitch1.update(pitchAcc1, -g1.gyro.y * RAD_TO_DEG, dt/1000.0);
        pitch2 = kalmanPitch2.update(pitchAcc2, -g2.gyro.y * RAD_TO_DEG, dt/1000.0);

        front_fleq = PLL(pitch1, &front_omega, &front_phi, dt);
        back_fleq = PLL(pitch2, &back_omega, &back_phi, dt);
        if(pitch1 <= front_deg + front_deg_range && pitch1 >= front_deg - front_deg_range && pitch2 <= front_deg + front_deg_range && pitch2 >= front_deg - front_deg_range){
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.setCursor(0,20);
            canvas.print("degree ready");
            canvas.setTextColor(GREEN,BLACK);
            canvas.drawCenterString(String(pitch1,3), M5.Display.width() / 2, M5.Display.height() / 2 -20);
            canvas.drawCenterString(String(pitch2,3), M5.Display.width() / 2, M5.Display.height() / 2 +20);
            canvas.pushSprite(0, 0);
        }else{
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.setCursor(0,20);
            canvas.print("degree ready");
            canvas.drawCenterString(String(pitch1,3), M5.Display.width() / 2, M5.Display.height() / 2 -20);
            canvas.drawCenterString(String(pitch2,3), M5.Display.width() / 2, M5.Display.height() / 2 +20);
            canvas.pushSprite(0, 0);
        }
    }
    Serial.println("GO");
    
    count = 0;

    SD_setup(BUFFER_SIZE);

    lsm1.getEvent(&a1, &mag1, &g1, &temp1);
    boolean g1_sign;
    if(g1.gyro.y >= 0.0) g1_sign = true;
    else g1_sign = false;

    // 割り込み設定
    // Initialize intrupt
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, control_cycle, true);
    timerAlarmEnable(timer);

    while(count < 2){
        while(Loop_flag == 0);
        Loop_flag = 0;

        lsm1.getEvent(&a1, &mag1, &g1, &temp1);
        lsm2.getEvent(&a2, &mag2, &g2, &temp2);
        pitchAcc1 = atan2(-a1.acceleration.x, sqrt(a1.acceleration.y * a1.acceleration.y + a1.acceleration.z * a1.acceleration.z)) * RAD_TO_DEG;
        pitchAcc2 = atan2(-a2.acceleration.x, sqrt(a2.acceleration.y * a2.acceleration.y + a2.acceleration.z * a2.acceleration.z)) * RAD_TO_DEG;
        pitch1 = kalmanPitch1.update(pitchAcc1, -g1.gyro.y * RAD_TO_DEG, 0.005);
        pitch2 = kalmanPitch2.update(pitchAcc2, -g2.gyro.y * RAD_TO_DEG, 0.005);

        front_fleq = PLL(pitch1, &front_omega, &front_phi, 5);
        back_fleq = PLL(pitch2, &back_omega, &back_phi, 5);

        degree_1 = -max_degree * cos(front_phi);
        degree_3 = -max_degree * cos(back_phi);

        logData[bufferCount].m1_torq = degree_1;
        logData[bufferCount].m2_torq = front_phi;
        logData[bufferCount].m3_torq = degree_3;
        logData[bufferCount].m4_torq = back_phi;

        logData[bufferCount].time_ms = millis();
        logData[bufferCount].front = radians(pitch1);
        logData[bufferCount].back = radians(pitch2);
        
        bufferCount++;

        if (bufferCount >= BUFFER_SIZE) {
            xQueueSend(sdQueue, logData, portMAX_DELAY);
            bufferCount = 0;
        }

        if(g1_sign == true && g1.gyro.y < 0.0){
            count++;
            g1_sign = false;
        }else if(g1_sign == false && g1.gyro.y >= 0.0){
            count++;
            g1_sign = true;
        }
    }

    while(true){
        while(Loop_flag == 0);
        Loop_flag = 0;
        M5.update();
        if(M5.BtnA.isPressed()){
            break;
        }

        lsm1.getEvent(&a1, &mag1, &g1, &temp1);
        lsm2.getEvent(&a2, &mag2, &g2, &temp2);
        pitchAcc1 = atan2(-a1.acceleration.x, sqrt(a1.acceleration.y * a1.acceleration.y + a1.acceleration.z * a1.acceleration.z)) * RAD_TO_DEG;
        pitchAcc2 = atan2(-a2.acceleration.x, sqrt(a2.acceleration.y * a2.acceleration.y + a2.acceleration.z * a2.acceleration.z)) * RAD_TO_DEG;
        pitch1 = kalmanPitch1.update(pitchAcc1, -g1.gyro.y * RAD_TO_DEG, 0.005);
        pitch2 = kalmanPitch2.update(pitchAcc2, -g2.gyro.y * RAD_TO_DEG, 0.005);

        front_fleq = PLL(pitch1, &front_omega, &front_phi, 5);
        back_fleq = PLL(pitch2, &back_omega, &back_phi, 5);

        degree_1 = -max_degree * cos(front_phi);
        degree_2 = -max_degree * cos(front_phi);
        degree_3 = -max_degree * cos(back_phi);
        degree_4 = -max_degree * cos(back_phi);

        tx_message.identifier = 0x101;
        tx_message.data[0] = 'P';
        ((float*)&tx_message.data[1])[0] = degree_1;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));

        tx_message.identifier = 0x102;
        tx_message.data[0] = 'P';
        ((float*)&tx_message.data[1])[0] = degree_2;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));

        tx_message.identifier = 0x103;
        tx_message.data[0] = 'P';
        ((float*)&tx_message.data[1])[0] = degree_3;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));

        tx_message.identifier = 0x104;
        tx_message.data[0] = 'P';
        ((float*)&tx_message.data[1])[0] = degree_4;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));

        while(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
            if(rx_message.identifier == 0x201){
                logData[bufferCount].m1_deg = *((float*)&rx_message.data[0]);
                logData[bufferCount].m1_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x202){
                logData[bufferCount].m2_deg = *((float*)&rx_message.data[0]);
                logData[bufferCount].m2_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x203){
                logData[bufferCount].m3_deg = *((float*)&rx_message.data[0]);
                logData[bufferCount].m3_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x204){
                logData[bufferCount].m4_deg = *((float*)&rx_message.data[0]);
                logData[bufferCount].m4_cur = *((float*)&rx_message.data[4]);
            }
        }
        logData[bufferCount].m1_torq = degree_1;
        logData[bufferCount].m2_torq = front_phi;
        logData[bufferCount].m3_torq = degree_3;
        logData[bufferCount].m4_torq = back_phi;

        logData[bufferCount].time_ms = millis();
        logData[bufferCount].front = radians(pitch1);
        logData[bufferCount].back = radians(pitch2);
        
        bufferCount++;

        if (bufferCount >= BUFFER_SIZE) {
            xQueueSend(sdQueue, logData, portMAX_DELAY);
            bufferCount = 0;
        }
    }
    timerAlarmDisable(timer);

    xQueueSend(sdQueue, logData, portMAX_DELAY);

    tx_message.identifier = 0x101;
    tx_message.data[0] = 'T';
    ((float*)&tx_message.data[1])[0] = 0.0;
    tx_message.data_length_code = 1 + sizeof(float); // 5
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    tx_message.identifier = 0x102;
    tx_message.data[0] = 'T';
    ((float*)&tx_message.data[1])[0] = 0.0;
    tx_message.data_length_code = 1 + sizeof(float); // 5
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    tx_message.identifier = 0x103;
    tx_message.data[0] = 'T';
    ((float*)&tx_message.data[1])[0] = 0.0;
    tx_message.data_length_code = 1 + sizeof(float); // 5
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    tx_message.identifier = 0x104;
    tx_message.data[0] = 'T';
    ((float*)&tx_message.data[1])[0] = 0.0;
    tx_message.data_length_code = 1 + sizeof(float); // 5
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    delay(1000);
    logFile.close();

    ESP.restart();
}

void trot_VirtualPassiveBase(void){
    const float front_deg = 30;
    const float front_deg_range = 3;

    float roll1,pitch1,yaw1;
    float roll2,pitch2,yaw2;
    SimpleKalmanFilter kalmanPitch1;
    SimpleKalmanFilter kalmanPitch2;
    sensors_event_t a1, mag1, g1, temp1;
    sensors_event_t a2, mag2, g2, temp2; 
    float pitchAcc1, pitchAcc2;

    const float tan_phi = tan(radians(5)); //仮想重力の傾きtanφ

    const int control_cycle = 5000; //制御周期(us)
    const int gear_rate = 16;
    const float KT = 0.150; //トルク定数(Nm/A)
    const float L1 = 0.3;
    const float m1 = 0.564;
    const float m2 = 0.378;
    const float m3 = 0.735;
    const float g = 9.81;
    const float u1 = 1.0f * L1 * (3*m1 + 4*m2 + m3) * g * tan_phi / KT / gear_rate;
    const float u2 = 1.0f * m1 * L1 * g * tan_phi / KT / gear_rate;
    float degree_1 = 0.0f, degree_2 = 0.0f, degree_3 = 0.0f, degree_4 = 0.0f;
    float torque_1 = 0.0f, torque_2 = 0.0f, torque_3 = 0.0f, torque_4 = 0.0f;
    float time = 0.0;
    int stand_f,stand_b; // 支持脚判定変数

    float torque_limit = 0.85f;
    float degree_limit_up = radians(15.0f); //遊脚
    float degree_limit_down = radians(7.8f); //支持脚

    boolean mainmode_flag = false;

    display_clear();
    display_println("main");

    const int BUFFER_SIZE = 10;
    int bufferCount = 0;     // バッファ内データ件数

    LogData logData[BUFFER_SIZE] = {0};

    if (!SD.begin(4)) {
        Serial.println("SDカードの初期化に失敗しました");
        while (M5.BtnB.isPressed() != true) M5.update();
    }

    String filename = makeUniqueFilename("/trot_VPB.csv");
    logFile = SD.open(filename, FILE_WRITE);
    if (!logFile) {
        Serial.println("ファイルオープン失敗");
        while (M5.BtnB.isPressed() != true) M5.update();
    }

    // ヘッダー
    logFile.println("time_ms,m1_deg,m1_cur,m1_torq,m2_deg,m2_cur,m2_torq,m3_deg,m3_cur,m3_torq,m4_deg,m4_cur,m4_torq,front,back");
    logFile.flush();

    // ドライバーリセット
    for(int i = 1; i <= 4; i++){
        tx_message.identifier = 0x100 + i;
        tx_message.data[0] = 'R';
        tx_message.data_length_code = 1;
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
        int count = 0;
        boolean mainmode_flag = false;
        do{
            delay(1);
            count++;
            if (count > 8000) { // タイムアウト処理
                Serial.print("timeout");
                mainmode_flag = true;
            }
            if(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
                if(rx_message.identifier == 0x200 + i && rx_message.data[0] == 'R'){
                    mainmode_flag = true;
                }
            }
        }while(mainmode_flag == false);
        Serial.printf("%#x:reset\n",0x100+i);
    }

    // キャリブレーション
    for(int i = 1; i <= 4; i++){
        tx_message.identifier = 0x100 + i;
        tx_message.data[0] = 'L';
        tx_message.data_length_code = 1;
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
        int count = 0;
        mainmode_flag = false;
        do{
            delay(1);
            count++;
            if (count > 5000) { // タイムアウト処理
                Serial.print("timeout");
                mainmode_flag = true;
            }
            if(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
                if(rx_message.identifier == 0x200 + i && rx_message.data[0] == 'L'){
                    mainmode_flag = true;
                }
            }
        }while(mainmode_flag == false);
        Serial.printf("%#x:calib\n",0x100+i);
    }

    // 制限設定
    for(int i = 1; i <= 4; i++){
        tx_message.identifier = 0x100 + i;
        tx_message.data[0] = 'B';
        tx_message.data[1] = 'T';
        ((float*)&tx_message.data[2])[0] = 0.8;
        tx_message.data_length_code = 2 + sizeof(float);
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
    }

    unsigned long lastTime = millis();
    int count = 0;
    while(count < 50){    //一定角度で安定するまで待機
        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0f;
        lastTime = now;
        lsm1.getEvent(&a1, &mag1, &g1, &temp1);
        lsm2.getEvent(&a2, &mag2, &g2, &temp2);
        pitchAcc1 = atan2(-a1.acceleration.x, sqrt(a1.acceleration.y * a1.acceleration.y + a1.acceleration.z * a1.acceleration.z)) * RAD_TO_DEG;
        pitchAcc2 = atan2(-a2.acceleration.x, sqrt(a2.acceleration.y * a2.acceleration.y + a2.acceleration.z * a2.acceleration.z)) * RAD_TO_DEG;
        pitch1 = kalmanPitch1.update(pitchAcc1, -g1.gyro.y * RAD_TO_DEG, dt);
        pitch2 = kalmanPitch2.update(pitchAcc2, -g2.gyro.y * RAD_TO_DEG, dt);
        if(pitch1 <= front_deg + front_deg_range && pitch1 >= front_deg - front_deg_range && pitch2 <= front_deg + front_deg_range && pitch2 >= front_deg - front_deg_range){
            count++;
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.setTextColor(GREEN,BLACK);
            canvas.drawCenterString(String(pitch1,3), M5.Display.width() / 2, M5.Display.height() / 2 -20);
            canvas.drawCenterString(String(pitch2,3), M5.Display.width() / 2, M5.Display.height() / 2 +20);
            canvas.pushSprite(0, 0);
        }else{
            count = 0;
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.drawCenterString(String(pitch1,3), M5.Display.width() / 2, M5.Display.height() / 2 -20);
            canvas.drawCenterString(String(pitch2,3), M5.Display.width() / 2, M5.Display.height() / 2 +20);
            canvas.pushSprite(0, 0);
        }
    }
    Serial.println("degree ready");
// */
    while(pitch1 >= 0){    //0°を跨ぐまで待機
        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0f;
        lastTime = now;
        lsm1.getEvent(&a1, &mag1, &g1, &temp1);
        lsm2.getEvent(&a2, &mag2, &g2, &temp2);
        pitchAcc1 = atan2(-a1.acceleration.x, sqrt(a1.acceleration.y * a1.acceleration.y + a1.acceleration.z * a1.acceleration.z)) * RAD_TO_DEG;
        pitchAcc2 = atan2(-a2.acceleration.x, sqrt(a2.acceleration.y * a2.acceleration.y + a2.acceleration.z * a2.acceleration.z)) * RAD_TO_DEG;
        pitch1 = kalmanPitch1.update(pitchAcc1, -g1.gyro.y * RAD_TO_DEG, dt);
        pitch2 = kalmanPitch2.update(pitchAcc2, -g2.gyro.y * RAD_TO_DEG, dt);
        if(pitch1 <= front_deg + front_deg_range && pitch1 >= front_deg - front_deg_range && pitch2 <= front_deg + front_deg_range && pitch2 >= front_deg - front_deg_range){
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.setCursor(0,20);
            canvas.print("degree ready");
            canvas.setTextColor(GREEN,BLACK);
            canvas.drawCenterString(String(pitch1,3), M5.Display.width() / 2, M5.Display.height() / 2 -20);
            canvas.drawCenterString(String(pitch2,3), M5.Display.width() / 2, M5.Display.height() / 2 +20);
            canvas.pushSprite(0, 0);
        }else{
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.setCursor(0,20);
            canvas.print("degree ready");
            canvas.drawCenterString(String(pitch1,3), M5.Display.width() / 2, M5.Display.height() / 2 -20);
            canvas.drawCenterString(String(pitch2,3), M5.Display.width() / 2, M5.Display.height() / 2 +20);
            canvas.pushSprite(0, 0);
        }
    }
    Serial.println("GO");

    lsm1.getEvent(&a1, &mag1, &g1, &temp1);
    lsm2.getEvent(&a2, &mag2, &g2, &temp2);
    if(-g1.gyro.y > 0){
        while(-g1.gyro.y >= 0){
            unsigned long now = millis();
            float dt = (now - lastTime) / 1000.0f;
            lastTime = now;
            lsm1.getEvent(&a1, &mag1, &g1, &temp1);
            lsm2.getEvent(&a2, &mag2, &g2, &temp2);
            pitchAcc1 = atan2(-a1.acceleration.x, sqrt(a1.acceleration.y * a1.acceleration.y + a1.acceleration.z * a1.acceleration.z)) * RAD_TO_DEG;
            pitchAcc2 = atan2(-a2.acceleration.x, sqrt(a2.acceleration.y * a2.acceleration.y + a2.acceleration.z * a2.acceleration.z)) * RAD_TO_DEG;
            pitch1 = kalmanPitch1.update(pitchAcc1, -g1.gyro.y * RAD_TO_DEG, dt);
            pitch2 = kalmanPitch2.update(pitchAcc2, -g2.gyro.y * RAD_TO_DEG, dt);
        }
    }else{
        while(-g1.gyro.y <= 0){
            unsigned long now = millis();
            float dt = (now - lastTime) / 1000.0f;
            lastTime = now;
            lsm1.getEvent(&a1, &mag1, &g1, &temp1);
            lsm2.getEvent(&a2, &mag2, &g2, &temp2);
            pitchAcc1 = atan2(-a1.acceleration.x, sqrt(a1.acceleration.y * a1.acceleration.y + a1.acceleration.z * a1.acceleration.z)) * RAD_TO_DEG;
            pitchAcc2 = atan2(-a2.acceleration.x, sqrt(a2.acceleration.y * a2.acceleration.y + a2.acceleration.z * a2.acceleration.z)) * RAD_TO_DEG;
            pitch1 = kalmanPitch1.update(pitchAcc1, -g1.gyro.y * RAD_TO_DEG, dt);
            pitch2 = kalmanPitch2.update(pitchAcc2, -g2.gyro.y * RAD_TO_DEG, dt);
        }
    }

    for(int i = 1; i <= 4; i++){
        tx_message.identifier = 0x100 + i;
        tx_message.data[0] = 'M';
        tx_message.data[1] = 'T';
        ((float*)&tx_message.data[2])[0] = 0.0;
        tx_message.data_length_code = 2 + sizeof(float); // 6
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
    }
    
    count = 0;


    SD_setup(BUFFER_SIZE);

    // 割り込み設定
    // Initialize intrupt
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, control_cycle, true);
    timerAlarmEnable(timer);

    while(true){
        // 割り込みにより200Hzで以降のコードが実行
        while (Loop_flag == 0);
        Loop_flag = 0;
        M5.update();
        if(M5.BtnA.isPressed()){
            break;
        }

        //kalman_update_2(&roll1,&pitch1,&yaw1,&roll2,&pitch2,&yaw2,0.005);
        lsm1.getEvent(&a1, &mag1, &g1, &temp1);
        lsm2.getEvent(&a2, &mag2, &g2, &temp2);
        pitchAcc1 = atan2(-a1.acceleration.x, sqrt(a1.acceleration.y * a1.acceleration.y + a1.acceleration.z * a1.acceleration.z)) * RAD_TO_DEG;
        pitchAcc2 = atan2(-a2.acceleration.x, sqrt(a2.acceleration.y * a2.acceleration.y + a2.acceleration.z * a2.acceleration.z)) * RAD_TO_DEG;
        pitch1 = kalmanPitch1.update(pitchAcc1, -g1.gyro.y * RAD_TO_DEG, 0.005);
        pitch2 = kalmanPitch2.update(pitchAcc2, -g2.gyro.y * RAD_TO_DEG, 0.005);

        // canvas.fillScreen(BLACK);
        // canvas.setCursor(0,0);
        // canvas.setTextSize(3);

        time += control_cycle/1000;

        stand_f = stand_judge(-radians(pitch1), degree_1, -degree_2);
        stand_b = stand_judge(radians(pitch2), degree_3, -degree_4);

        if(stand_f == 1){   // 右(m1)が支持脚
            torque_1 = -u1 * cos(degree_1);
            torque_2 = -u2 * cos(degree_2);
            // canvas.setTextColor(WHITE,BLACK);
        }else if(stand_f == 2){ // 左(m2)が支持脚
            torque_1 = u2 * cos(degree_1);
            torque_2 = u1 * cos(degree_2);
            // canvas.setTextColor(GREEN,BLACK);
        }else{
            torque_1 = 0.0;
            torque_2 = 0.0;
            // canvas.setTextColor(RED,BLACK);
        }
        // canvas.drawCenterString(String(pitch1,3), M5.Display.width() / 2, M5.Display.height() / 2 -20);
        if(stand_b == 1){ // 右(m3)が支持脚
            torque_3 = -u1 * cos(degree_3);
            torque_4 = -u2 * cos(degree_4) * 2.0;
            // canvas.setTextColor(GREEN,BLACK);
        }else if(stand_b == 2){ // 左(m4)が支持脚
            torque_3 = u2 * cos(degree_3);
            torque_4 = u1 * cos(degree_4) * 2.0;
            // canvas.setTextColor(WHITE,BLACK);
        }else{
            torque_3 = 0.0;
            torque_4 = 0.0;
            // canvas.setTextColor(RED,BLACK);
        }
        // canvas.drawCenterString(String(pitch2,3), M5.Display.width() / 2, M5.Display.height() / 2 +20);
        // canvas.pushSprite(0, 0);

        if(isfinite(torque_1) == false) torque_1 = 0.0;
        if(isfinite(torque_2) == false) torque_2 = 0.0;
        if(isfinite(torque_3) == false) torque_3 = 0.0;
        if(isfinite(torque_4) == false) torque_4 = 0.0;

        tx_message.identifier = 0x101;
        if(degree_1 >= degree_limit_up-radians(0.5) && stand_f == 2) { //遊脚
            tx_message.data[0] = 'P';
            ((float*)&tx_message.data[1])[0] = degree_limit_up;
        }else if(degree_1 <= -degree_limit_down+radians(0.5) && stand_f == 1) { //支持脚
            tx_message.data[0] = 'P';
            ((float*)&tx_message.data[1])[0] = -degree_limit_down;
        }else{
            tx_message.data[0] = 'T';
            ((float*)&tx_message.data[1])[0] = torque_1;
        }
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));
        //ets_delay_us(200);  // 200µs

        tx_message.identifier = 0x102;
        if(degree_2 >= degree_limit_down-radians(0.5) && stand_f == 2) { //支持脚
            tx_message.data[0] = 'P';
            ((float*)&tx_message.data[1])[0] = degree_limit_down;
        }else if(degree_2 <= -degree_limit_up+radians(0.5) && stand_f == 1) {
            tx_message.data[0] = 'P';
            ((float*)&tx_message.data[1])[0] = -degree_limit_up;
        }else{
            tx_message.data[0] = 'T';
            ((float*)&tx_message.data[1])[0] = torque_2;
        }
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));
        //ets_delay_us(200);  // 200µs

        tx_message.identifier = 0x103;
        if(degree_3 >= degree_limit_up-radians(0.5) && stand_b == 2) { //遊脚
            tx_message.data[0] = 'P';
            ((float*)&tx_message.data[1])[0] = degree_limit_up;
        }else if(degree_3 <= -degree_limit_down+radians(0.5) && stand_b == 1) { //支持脚
            tx_message.data[0] = 'P';
            ((float*)&tx_message.data[1])[0] = -degree_limit_down;
        }else{
            tx_message.data[0] = 'T';
            ((float*)&tx_message.data[1])[0] = torque_3;
        }
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));
        //ets_delay_us(200);  // 200µs

        tx_message.identifier = 0x104;
        if(degree_4 >= degree_limit_down-radians(0.5) && stand_b == 2) { //支持脚
            tx_message.data[0] = 'P';
            ((float*)&tx_message.data[1])[0] = degree_limit_down;
        }else if(degree_4 <= -degree_limit_up+radians(0.5) && stand_b == 1) {
            tx_message.data[0] = 'P';
            ((float*)&tx_message.data[1])[0] = -degree_limit_up;
        }else{
            tx_message.data[0] = 'T';
            ((float*)&tx_message.data[1])[0] = torque_4;
        }
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));
        //ets_delay_us(200);  // 200µs

        logData[bufferCount] = {0};

        logData[bufferCount].m1_torq = torque_1;
        logData[bufferCount].m2_torq = torque_2;
        logData[bufferCount].m3_torq = torque_3;
        logData[bufferCount].m4_torq = torque_4;

        while(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
            if(rx_message.identifier == 0x201){
                logData[bufferCount].m1_deg = *((float*)&rx_message.data[0]);
                degree_1 = logData[bufferCount].m1_deg;
                logData[bufferCount].m1_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x202){
                logData[bufferCount].m2_deg = *((float*)&rx_message.data[0]);
                degree_2 = logData[bufferCount].m2_deg;
                logData[bufferCount].m2_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x203){
                logData[bufferCount].m3_deg = *((float*)&rx_message.data[0]);
                degree_3 = logData[bufferCount].m3_deg;
                logData[bufferCount].m3_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x204){
                logData[bufferCount].m4_deg = *((float*)&rx_message.data[0]);
                degree_4 = logData[bufferCount].m4_deg;
                logData[bufferCount].m4_cur = *((float*)&rx_message.data[4]);
            }
        }

        logData[bufferCount].time_ms = millis();
        logData[bufferCount].front = radians(pitch1);
        logData[bufferCount].back = radians(pitch2);

        bufferCount++;

        if (bufferCount >= BUFFER_SIZE) {
            xQueueSend(sdQueue, logData, portMAX_DELAY);
            bufferCount = 0;
        }
    }
    timerAlarmDisable(timer);

    xQueueSend(sdQueue, logData, portMAX_DELAY);

    tx_message.identifier = 0x101;
    tx_message.data[0] = 'E';
    tx_message.data[1] = '0';
    tx_message.data_length_code = 2;
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    tx_message.identifier = 0x102;
    tx_message.data[0] = 'E';
    tx_message.data[1] = '0';
    tx_message.data_length_code = 2;
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    tx_message.identifier = 0x103;
    tx_message.data[0] = 'E';
    tx_message.data[1] = '0';
    tx_message.data_length_code = 2;
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    tx_message.identifier = 0x104;
    tx_message.data[0] = 'E';
    tx_message.data[1] = '0';
    tx_message.data_length_code = 2;
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    delay(1000);
    logFile.close();

    ESP.restart();
}

void walk_VirtualPassiveBase(void){

}

void test2_fix(void){
    const float front_deg = 20;
    const float front_deg_range = 3;

    float roll1,pitch1,yaw1;
    float roll2,pitch2,yaw2;
    SimpleKalmanFilter kalmanPitch1;
    SimpleKalmanFilter kalmanPitch2;
    sensors_event_t a1, mag1, g1, temp1;
    sensors_event_t a2, mag2, g2, temp2; 
    float pitchAcc1, pitchAcc2;

    float front_fleq,back_fleq;
    float front_omega = 0.0, back_omega = 0.0;
    float front_phi = 0.0, back_phi = 0.0;
    int front_lastcross, back_lastcross;

    const int control_cycle = 5000; //制御周期(us)
    const int gear_rate = 16;
    float max_degree = radians(7.8); // 最大角度
    const float degree_delta = radians(3.0); // 後ろにずらす角度
    float speed1 = 15;  // 速度目標（高い方）
    float speed2 = 5;   // 速度目標（低い方）

    float degree_1,degree_2,degree_3,degree_4;
    float old_degree = 0.0;
    float time = 0.0;

    boolean mainmode_flag = false;

    display_clear();
    display_println("main");

    const int BUFFER_SIZE = 10;
    // String dataBuffer = "";  // 書き込み用バッファ
    int bufferCount = 0;     // バッファ内データ件数
    // float m1deg,m2deg,m3deg,m4deg = 0.0;
    // float m1cur,m2cur,m3cur,m4cur = 0.0;

    LogData logData[BUFFER_SIZE] = {0};

    if (!SD.begin(4)) {
        Serial.println("SDカードの初期化に失敗しました");
        while (1);
    }

    String filename = makeUniqueFilename("/trot_fixCycle.csv");
    logFile = SD.open(filename, FILE_WRITE);
    if (!logFile) {
        Serial.println("ファイルオープン失敗");
        while (1);
    }

    // ヘッダー
    logFile.println("time_ms,m1_deg,m1_cur,,m2_deg,m2_cur,front_phi,m3_deg,m3_cur,,m4_deg,m4_cur,back_phi,front,back");
    logFile.flush();

    // ドライバーリセット
    for(int i = 1; i <= 4; i++){
        tx_message.identifier = 0x100 + i;
        tx_message.data[0] = 'R';
        tx_message.data_length_code = 1;
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
        int count = 0;
        boolean mainmode_flag = false;
        do{
            delay(1);
            count++;
            if (count > 8000) { // タイムアウト処理
                Serial.print("timeout");
                mainmode_flag = true;
            }
            if(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
                if(rx_message.identifier == 0x200 + i && rx_message.data[0] == 'R'){
                    mainmode_flag = true;
                }
            }
        }while(mainmode_flag == false);
        Serial.printf("%#x:reset\n",0x100+i);
    }

    // キャリブレーション
    for(int i = 1; i <= 4; i++){
        tx_message.identifier = 0x100 + i;
        tx_message.data[0] = 'L';
        tx_message.data_length_code = 1;
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
        int count = 0;
        mainmode_flag = false;
        do{
            delay(1);
            count++;
            if (count > 5000) { // タイムアウト処理
                Serial.print("timeout");
                mainmode_flag = true;
            }
            if(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
                if(rx_message.identifier == 0x200 + i && rx_message.data[0] == 'L'){
                    mainmode_flag = true;
                }
            }
        }while(mainmode_flag == false);
        Serial.printf("%#x:calib\n",0x100+i);
    }

    unsigned long lastTime = millis();
    int count = 0;
    while(count < 50){    //一定角度で安定するまで待機
        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0f;
        lastTime = now;
        lsm1.getEvent(&a1, &mag1, &g1, &temp1);
        lsm2.getEvent(&a2, &mag2, &g2, &temp2);
        pitchAcc1 = atan2(-a1.acceleration.x, sqrt(a1.acceleration.y * a1.acceleration.y + a1.acceleration.z * a1.acceleration.z)) * RAD_TO_DEG;
        pitchAcc2 = atan2(-a2.acceleration.x, sqrt(a2.acceleration.y * a2.acceleration.y + a2.acceleration.z * a2.acceleration.z)) * RAD_TO_DEG;
        pitch1 = kalmanPitch1.update(pitchAcc1, -g1.gyro.y * RAD_TO_DEG, dt);
        pitch2 = kalmanPitch2.update(pitchAcc2, -g2.gyro.y * RAD_TO_DEG, dt);
        if(pitch1 <= front_deg + front_deg_range && pitch1 >= front_deg - front_deg_range && pitch2 <= front_deg + front_deg_range && pitch2 >= front_deg - front_deg_range){
            count++;
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.setTextColor(GREEN,BLACK);
            canvas.drawCenterString(String(pitch1,3), M5.Display.width() / 2, M5.Display.height() / 2 -20);
            canvas.drawCenterString(String(pitch2,3), M5.Display.width() / 2, M5.Display.height() / 2 +20);
            canvas.pushSprite(0, 0);
        }else{
            count = 0;
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.drawCenterString(String(pitch1,3), M5.Display.width() / 2, M5.Display.height() / 2 -20);
            canvas.drawCenterString(String(pitch2,3), M5.Display.width() / 2, M5.Display.height() / 2 +20);
            canvas.pushSprite(0, 0);
        }
    }
    Serial.println("degree ready");
// */
    while(pitch1 >= 0){    //0°を跨ぐまで待機
        unsigned long now = millis();
        float dt = now - lastTime;
        lastTime = now;
        lsm1.getEvent(&a1, &mag1, &g1, &temp1);
        lsm2.getEvent(&a2, &mag2, &g2, &temp2);
        pitchAcc1 = atan2(-a1.acceleration.x, sqrt(a1.acceleration.y * a1.acceleration.y + a1.acceleration.z * a1.acceleration.z)) * RAD_TO_DEG;
        pitchAcc2 = atan2(-a2.acceleration.x, sqrt(a2.acceleration.y * a2.acceleration.y + a2.acceleration.z * a2.acceleration.z)) * RAD_TO_DEG;
        pitch1 = kalmanPitch1.update(pitchAcc1, -g1.gyro.y * RAD_TO_DEG, dt/1000.0);
        pitch2 = kalmanPitch2.update(pitchAcc2, -g2.gyro.y * RAD_TO_DEG, dt/1000.0);

        front_fleq = PLL(pitch1, &front_omega, &front_phi, dt);
        back_fleq = PLL(pitch2, &back_omega, &back_phi, dt);
        if(pitch1 <= front_deg + front_deg_range && pitch1 >= front_deg - front_deg_range && pitch2 <= front_deg + front_deg_range && pitch2 >= front_deg - front_deg_range){
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.setCursor(0,20);
            canvas.print("degree ready");
            canvas.setTextColor(GREEN,BLACK);
            canvas.drawCenterString(String(pitch1,3), M5.Display.width() / 2, M5.Display.height() / 2 -20);
            canvas.drawCenterString(String(pitch2,3), M5.Display.width() / 2, M5.Display.height() / 2 +20);
            canvas.pushSprite(0, 0);
        }else{
            canvas.fillScreen(BLACK);
            canvas.setTextSize(2);
            canvas.setTextColor(WHITE,BLACK);
            canvas.setCursor(0,0);
            canvas.print("main");
            canvas.setTextSize(3);
            canvas.setCursor(0,20);
            canvas.print("degree ready");
            canvas.drawCenterString(String(pitch1,3), M5.Display.width() / 2, M5.Display.height() / 2 -20);
            canvas.drawCenterString(String(pitch2,3), M5.Display.width() / 2, M5.Display.height() / 2 +20);
            canvas.pushSprite(0, 0);
        }
    }
    Serial.println("GO");
    
    count = 0;

    SD_setup(BUFFER_SIZE);

    lsm1.getEvent(&a1, &mag1, &g1, &temp1);
    boolean g1_sign;
    if(g1.gyro.y >= 0.0) g1_sign = true;
    else g1_sign = false;

    // 割り込み設定
    // Initialize intrupt
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, control_cycle, true);
    timerAlarmEnable(timer);

    while(count < 2){
        while(Loop_flag == 0);
        Loop_flag = 0;

        lsm1.getEvent(&a1, &mag1, &g1, &temp1);
        lsm2.getEvent(&a2, &mag2, &g2, &temp2);
        pitchAcc1 = atan2(-a1.acceleration.x, sqrt(a1.acceleration.y * a1.acceleration.y + a1.acceleration.z * a1.acceleration.z)) * RAD_TO_DEG;
        pitchAcc2 = atan2(-a2.acceleration.x, sqrt(a2.acceleration.y * a2.acceleration.y + a2.acceleration.z * a2.acceleration.z)) * RAD_TO_DEG;
        pitch1 = kalmanPitch1.update(pitchAcc1, -g1.gyro.y * RAD_TO_DEG, 0.005);
        pitch2 = kalmanPitch2.update(pitchAcc2, -g2.gyro.y * RAD_TO_DEG, 0.005);

        front_fleq = PLL(pitch1, &front_omega, &front_phi, 5);
        back_fleq = PLL(pitch2, &back_omega, &back_phi, 5);

        max_degree = 2;

        degree_1 = -max_degree * cos(front_phi);
        degree_3 = -max_degree * cos(back_phi);

        logData[bufferCount].m1_torq = degree_1;
        logData[bufferCount].m2_torq = front_phi;
        logData[bufferCount].m3_torq = degree_3;
        logData[bufferCount].m4_torq = back_phi;

        logData[bufferCount].time_ms = millis();
        logData[bufferCount].front = radians(pitch1);
        logData[bufferCount].back = radians(pitch2);
        
        bufferCount++;

        if (bufferCount >= BUFFER_SIZE) {
            xQueueSend(sdQueue, logData, portMAX_DELAY);
            bufferCount = 0;
        }

        if(g1_sign == true && g1.gyro.y < 0.0){
            count++;
            g1_sign = false;
        }else if(g1_sign == false && g1.gyro.y >= 0.0){
            count++;
            g1_sign = true;
        }
    }

    while(true){
        while(Loop_flag == 0);
        Loop_flag = 0;
        M5.update();
        if(M5.BtnA.isPressed()){
            break;
        }

        lsm1.getEvent(&a1, &mag1, &g1, &temp1);
        lsm2.getEvent(&a2, &mag2, &g2, &temp2);
        pitchAcc1 = atan2(-a1.acceleration.x, sqrt(a1.acceleration.y * a1.acceleration.y + a1.acceleration.z * a1.acceleration.z)) * RAD_TO_DEG;
        pitchAcc2 = atan2(-a2.acceleration.x, sqrt(a2.acceleration.y * a2.acceleration.y + a2.acceleration.z * a2.acceleration.z)) * RAD_TO_DEG;
        pitch1 = kalmanPitch1.update(pitchAcc1, -g1.gyro.y * RAD_TO_DEG, 0.005);
        pitch2 = kalmanPitch2.update(pitchAcc2, -g2.gyro.y * RAD_TO_DEG, 0.005);

        front_fleq = PLL(pitch1, &front_omega, &front_phi, 5);
        back_fleq = PLL(pitch2, &back_omega, &back_phi, 5);

        degree_1 = -max_degree * cos(front_phi);
        degree_2 = -max_degree * cos(front_phi);
        degree_3 = -max_degree * cos(back_phi);
        degree_4 = -max_degree * cos(back_phi);

        tx_message.identifier = 0x101;
        tx_message.data[0] = 'P';
        ((float*)&tx_message.data[1])[0] = degree_1;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));

        tx_message.identifier = 0x102;
        tx_message.data[0] = 'P';
        ((float*)&tx_message.data[1])[0] = degree_2;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));

        tx_message.identifier = 0x103;
        tx_message.data[0] = 'P';
        ((float*)&tx_message.data[1])[0] = degree_3;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));

        tx_message.identifier = 0x104;
        tx_message.data[0] = 'P';
        ((float*)&tx_message.data[1])[0] = degree_4;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));

        while(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
            if(rx_message.identifier == 0x201){
                logData[bufferCount].m1_deg = *((float*)&rx_message.data[0]);
                logData[bufferCount].m1_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x202){
                logData[bufferCount].m2_deg = *((float*)&rx_message.data[0]);
                logData[bufferCount].m2_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x203){
                logData[bufferCount].m3_deg = *((float*)&rx_message.data[0]);
                logData[bufferCount].m3_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x204){
                logData[bufferCount].m4_deg = *((float*)&rx_message.data[0]);
                logData[bufferCount].m4_cur = *((float*)&rx_message.data[4]);
            }
        }
        logData[bufferCount].m1_torq = degree_1;
        logData[bufferCount].m2_torq = front_phi;
        logData[bufferCount].m3_torq = degree_3;
        logData[bufferCount].m4_torq = back_phi;

        logData[bufferCount].time_ms = millis();
        logData[bufferCount].front = radians(pitch1);
        logData[bufferCount].back = radians(pitch2);
        
        bufferCount++;

        if (bufferCount >= BUFFER_SIZE) {
            xQueueSend(sdQueue, logData, portMAX_DELAY);
            bufferCount = 0;
        }
    }
    timerAlarmDisable(timer);

    xQueueSend(sdQueue, logData, portMAX_DELAY);

    tx_message.identifier = 0x101;
    tx_message.data[0] = 'T';
    ((float*)&tx_message.data[1])[0] = 0.0;
    tx_message.data_length_code = 1 + sizeof(float); // 5
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    tx_message.identifier = 0x102;
    tx_message.data[0] = 'T';
    ((float*)&tx_message.data[1])[0] = 0.0;
    tx_message.data_length_code = 1 + sizeof(float); // 5
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    tx_message.identifier = 0x103;
    tx_message.data[0] = 'T';
    ((float*)&tx_message.data[1])[0] = 0.0;
    tx_message.data_length_code = 1 + sizeof(float); // 5
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    tx_message.identifier = 0x104;
    tx_message.data[0] = 'T';
    ((float*)&tx_message.data[1])[0] = 0.0;
    tx_message.data_length_code = 1 + sizeof(float); // 5
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    delay(1000);
    logFile.close();

    ESP.restart();
}

void test2_VPB(void){

}

void test3_fix(void){

}

void test3_VPB(void){

}

void torqueMode(void) {
    resetMotor(); // モーターをリセット

    tx_message.data[0] = 'M'; // モード変更コマンド
    tx_message.data[1] = 'T'; // トルク制御モード
    ((float*)&tx_message.data[2])[0] = 0.0; // トルクの初期値
    tx_message.data_length_code = 2 + sizeof(float); // データ長
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
    display_println("Torque Mode Set 0.0");

    sendStart(); // モーターを開始
    display_println("Motor Started");
    delay(3000); // 3秒待機
}

void speedMode(void) {
    resetMotor(); // モーターをリセット

    tx_message.data[0] = 'M'; // モード変更コマンド
    tx_message.data[1] = 'S'; // 速度制御モード
    ((float*)&tx_message.data[2])[0] = 0.0; // 速度の初期値
    tx_message.data_length_code = 2 + sizeof(float); // データ長
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
    display_println("Speed Mode Set 0.0");
    delay(1000);

    sendStart(); // モーターを開始
    display_println("Motor Started");
    delay(3000); // 3秒待機
}

void positionMode(void) {
    resetMotor(); // モーターをリセット

    tx_message.data[0] = 'M'; // モード変更コマンド
    tx_message.data[1] = 'P'; // 位置制御モード
    ((float*)&tx_message.data[2])[0] = 0.0; // 位置の初期値
    tx_message.data_length_code = 2 + sizeof(float); // データ長
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
    display_println("Position Mode Set 0.0");
    delay(1000);

    sendStart(); // モーターを開始
    display_println("Motor Started");
    delay(3000); // 3秒待機
}

void torqueCalibration(void){
    const int control_cycle = 5000; //制御周期(us)
    const int gear_rate = 16;
    const float max_degree = radians(45);

    float degree_1,degree_2,degree_3,degree_4;
    float old_degree = 0.0;
    float time = 0.0;

    boolean mainmode_flag = false;

    display_clear();
    display_println("torque calib");

    const int BUFFER_SIZE = 10;
    int bufferCount = 0;     // バッファ内データ件数

    LogData logData[BUFFER_SIZE] = {0};

    if (!SD.begin(4)) {
        Serial.println("SDカードの初期化に失敗しました");
        while (1);
    }

    String filename = makeUniqueFilename("/torque_calib.csv");
    logFile = SD.open(filename, FILE_WRITE);
    if (!logFile) {
        Serial.println("ファイルオープン失敗");
        while (1);
    }

    // ヘッダー
    logFile.println("time_ms,m1_deg,m1_cur,m2_deg,m2_cur,m3_deg,m3_cur,m4_deg,m4_cur,front,back");
    logFile.flush();

    // ドライバーリセット
    for(int i = 1; i <= 4; i++){
        tx_message.identifier = 0x100 + i;
        tx_message.data[0] = 'R';
        tx_message.data_length_code = 1;
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
        int count = 0;
        boolean mainmode_flag = false;
        do{
            delay(1);
            count++;
            if (count > 8000) { // タイムアウト処理
                Serial.print("timeout");
                mainmode_flag = true;
            }
            if(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
                if(rx_message.identifier == 0x200 + i && rx_message.data[0] == 'R'){
                    mainmode_flag = true;
                }
            }
        }while(mainmode_flag == false);
        Serial.printf("%#x:reset\n",0x100+i);
    }

    // キャリブレーション
    for(int i = 1; i <= 4; i++){
        tx_message.identifier = 0x100 + i;
        tx_message.data[0] = 'L';
        tx_message.data_length_code = 1;
        if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
        int count = 0;
        mainmode_flag = false;
        do{
            delay(1);
            count++;
            if (count > 5000) { // タイムアウト処理
                Serial.print("timeout");
                mainmode_flag = true;
            }
            if(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
                if(rx_message.identifier == 0x200 + i && rx_message.data[0] == 'L'){
                    mainmode_flag = true;
                }
            }
        }while(mainmode_flag == false);
        Serial.printf("%#x:calib\n",0x100+i);
    }

    SD_setup(BUFFER_SIZE);

    // 割り込み設定
    // Initialize intrupt
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, control_cycle, true);
    timerAlarmEnable(timer);

    while(true){
        // 割り込みにより200Hzで以降のコードが実行
        while (Loop_flag == 0);
        Loop_flag = 0;
        M5.update();
        if(M5.BtnA.isPressed()){
            break;
        }

        time += control_cycle/1000;
        degree_1 = radians(-90);
        degree_2 = radians(90);
        degree_3 = radians(90);
        degree_4 = radians(-90);

        tx_message.identifier = 0x101;
        tx_message.data[0] = 'P';
        ((float*)&tx_message.data[1])[0] = degree_1;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));
        //ets_delay_us(200);  // 200µs

        tx_message.identifier = 0x102;
        tx_message.data[0] = 'P';
        ((float*)&tx_message.data[1])[0] = degree_2;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));
        //ets_delay_us(200);  // 200µs

        tx_message.identifier = 0x103;
        tx_message.data[0] = 'P';
        ((float*)&tx_message.data[1])[0] = degree_3;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));
        //ets_delay_us(200);  // 200µs

        tx_message.identifier = 0x104;
        tx_message.data[0] = 'P';
        ((float*)&tx_message.data[1])[0] = degree_4;
        tx_message.data_length_code = 1 + sizeof(float); // 5
        twai_transmit(&tx_message, pdMS_TO_TICKS(5));
        //ets_delay_us(200);  // 200µs

        while(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
            if(rx_message.identifier == 0x201){
                logData[bufferCount].m1_deg = *((float*)&rx_message.data[0]);
                logData[bufferCount].m1_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x202){
                logData[bufferCount].m2_deg = *((float*)&rx_message.data[0]);
                logData[bufferCount].m2_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x203){
                logData[bufferCount].m3_deg = *((float*)&rx_message.data[0]);
                logData[bufferCount].m3_cur = *((float*)&rx_message.data[4]);
            }else if(rx_message.identifier == 0x204){
                logData[bufferCount].m4_deg = *((float*)&rx_message.data[0]);
                logData[bufferCount].m4_cur = *((float*)&rx_message.data[4]);
            }
        }

        logData[bufferCount].time_ms = millis();
        bufferCount++;

        if (bufferCount >= BUFFER_SIZE) {
            xQueueSend(sdQueue, logData, portMAX_DELAY);
            bufferCount = 0;
        }

        old_degree = degree_1;
    }
    timerAlarmDisable(timer);

    xQueueSend(sdQueue, logData, portMAX_DELAY);

    tx_message.identifier = 0x101;
    tx_message.data[0] = 'P';
    ((float*)&tx_message.data[1])[0] = 0.0;
    tx_message.data_length_code = 1 + sizeof(float); // 5
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    tx_message.identifier = 0x102;
    tx_message.data[0] = 'P';
    ((float*)&tx_message.data[1])[0] = 0.0;
    tx_message.data_length_code = 1 + sizeof(float); // 5
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    tx_message.identifier = 0x103;
    tx_message.data[0] = 'P';
    ((float*)&tx_message.data[1])[0] = 0.0;
    tx_message.data_length_code = 1 + sizeof(float); // 5
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    tx_message.identifier = 0x104;
    tx_message.data[0] = 'P';
    ((float*)&tx_message.data[1])[0] = 0.0;
    tx_message.data_length_code = 1 + sizeof(float); // 5
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);

    delay(1000);
    logFile.close();

    ESP.restart();
}

void sendStop(void) {
    tx_message.data[0] = 'E'; // 停止コマンド
    tx_message.data[1] = '0'; // 停止フラグ
    tx_message.data_length_code = 2; // データ長
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
}

void sendStart(void) {
    tx_message.data[0] = 'E'; // 動作開始コマンド
    tx_message.data[1] = '1'; // 動作フラグ
    tx_message.data_length_code = 2; // データ長
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
}

void resetMotor(void) {
    display_clear(); // ディスプレイをクリア

    tx_message.identifier = 0x201;
    tx_message.data[0] = 'R';
    tx_message.data_length_code = 1;
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK) {
        //Serial.println("CAN Message Sent!");
    } else {
        //Serial.println("CAN Send Failed");
    }
    Serial.println("Motor Restart");
    int count = 0;
    boolean cantest_flag = false;
    do{
        delay(1);
        count++;
        if (count > 10000) { // タイムアウト処理
            display_println("Timeout waiting for 'R'");
            delay(1000);
            cantest_flag = true;
        }
        if(twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK){
            if(rx_message.identifier == 0x101 && rx_message.data[0] == 'R'){
                cantest_flag = true;
            }
        }
    }while(cantest_flag == false);
}

void calibration(int id){
    tx_message.identifier = id;

    resetMotor(); // モーターをリセット

    tx_message.data[0] = 'L'; // キャリブレーションコマンド
    tx_message.data_length_code = 1; // データ長
    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
}

void uart2CAN(void) {
    serial_input = ""; // シリアル入力を初期化
    serial_value = 0.0; // シリアル値を初期化
    switch(Serial.read()){
        case 'I':
            while(Serial.available()){
                char c = Serial.read();
                if(c == '\n'){

                }else{
                    serial_input += c; // シリアルからの入力を連結
                }
            }
            serial_input.trim();
            tx_message.identifier = (int) strtol(serial_input.c_str(), NULL, 16);
            Serial.printf("Read ID : %s\n", serial_input.c_str());
            Serial.printf("Set ID : %#x\n", tx_message.identifier);
            break;
        case 'A':
            tx_message.data[0] = 'A'; // 角度取得コマンド
            tx_message.data_length_code = 1;
            if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
            break;
        case 'V':
            tx_message.data[0] = 'V'; // 速度取得コマンド
            tx_message.data_length_code = 1;
            if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
            break;
        case 'C':
            tx_message.data[0] = 'C'; // 電流取得コマンド
            tx_message.data_length_code = 1;
            if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
            break;
        case 'T': // トルク設定
            while(Serial.available()){
                char c = Serial.read();
                if(c == '\n'){
                    serial_value = serial_input.substring(0).toFloat();
                }else{
                    serial_input += c; // シリアルからの入力を連結
                }
            }
            tx_message.data[0] = 'T';
            ((float*)&tx_message.data[1])[0] = serial_input.substring(0).toFloat();
            tx_message.data_length_code = 1 + sizeof(float);
            if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
            break;
        case 'S': // 速度設定
            while(Serial.available()){
                char c = Serial.read();
                if(c == '\n'){
                    serial_value = serial_input.substring(0).toFloat();
                }else{
                    serial_input += c; // シリアルからの入力を連結
                }
            }
            serial_input.trim();
            tx_message.data[0] = 'S';
            ((float*)&tx_message.data[1])[0] = serial_input.substring(0).toFloat();
            tx_message.data_length_code = 1 + sizeof(float);
            if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
            break;
        case 'P': // 位置設定
            while(Serial.available()){
                char c = Serial.read();
                if(c == '\n'){
                    serial_value = serial_input.substring(0).toFloat();
                }else{
                    serial_input += c; // シリアルからの入力を連結
                }
            }
            serial_input.trim();
            tx_message.data[0] = 'P';
            ((float*)&tx_message.data[1])[0] = serial_input.substring(0).toFloat();
            tx_message.data_length_code = 1 + sizeof(float);
            if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
            break;
        case 'E': // ドライバーON/OFF
            tx_message.data[0] = 'E';
            tx_message.data[1] = Serial.read();
            tx_message.data_length_code = 2;
            if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
            break;
        case 'L': // キャリブレーション
            tx_message.data[0] = 'L';
            tx_message.data_length_code = 1;
            if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
            break;
        case 'M': // モード変更
            switch(Serial.read()){
                case 'T':
                    while(Serial.available()){
                        char c = Serial.read();
                        if(c == '\n'){
                            serial_value = serial_input.substring(0).toFloat();
                        }else{
                            serial_input += c; // シリアルからの入力を連結
                        }
                    }
                    serial_input.trim();
                    tx_message.data[0] = 'M';
                    tx_message.data[1] = 'T';
                    ((float*)&tx_message.data[2])[0] = serial_input.substring(0).toFloat();
                    tx_message.data_length_code = 2 + sizeof(float);
                    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
                    break;
                case 'S':
                    while(Serial.available()){
                        char c = Serial.read();
                        if(c == '\n'){
                            serial_value = serial_input.substring(0).toFloat();
                        }else{
                            serial_input += c; // シリアルからの入力を連結
                        }
                    }
                    serial_input.trim();
                    tx_message.data[0] = 'M';
                    tx_message.data[1] = 'S';
                    ((float*)&tx_message.data[2])[0] = serial_input.substring(0).toFloat();
                    tx_message.data_length_code = 2 + sizeof(float);
                    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
                    break;
                case 'P':
                    while(Serial.available()){
                        char c = Serial.read();
                        if(c == '\n'){
                            serial_value = serial_input.substring(0).toFloat();
                        }else{
                            serial_input += c; // シリアルからの入力を連結
                        }
                    }
                    tx_message.data[0] = 'M';
                    tx_message.data[1] = 'P';
                    ((float*)&tx_message.data[2])[0] = serial_input.substring(0).toFloat();
                    tx_message.data_length_code = 2 + sizeof(float);
                    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
                    break;
                default:
                    break;
            }
            break;
        case 'B': // 制限設定
            switch(Serial.read()){
                case 'T':
                    while(Serial.available()){
                        char c = Serial.read();
                        if(c == '\n'){
                            serial_value = serial_input.substring(0).toFloat();
                        }else{
                            serial_input += c; // シリアルからの入力を連結
                        }
                    }
                    serial_input.trim();
                    tx_message.data[0] = 'B';
                    tx_message.data[1] = 'T';
                    ((float*)&tx_message.data[2])[0] = serial_input.substring(0).toFloat();
                    tx_message.data_length_code = 2 + sizeof(float);
                    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
                    break;
                case 'V':
                    while(Serial.available()){
                        char c = Serial.read();
                        if(c == '\n'){
                            serial_value = serial_input.substring(0).toFloat();
                        }else{
                            serial_input += c; // シリアルからの入力を連結
                        }
                    }
                    serial_input.trim();
                    tx_message.data[0] = 'B';
                    tx_message.data[1] = 'V';
                    ((float*)&tx_message.data[2])[0] = serial_input.substring(0).toFloat();
                    tx_message.data_length_code = 2 + sizeof(float);
                    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
                    break;
                case 'A':
                    while(Serial.available()){
                        char c = Serial.read();
                        if(c == '\n'){
                            serial_value = serial_input.substring(0).toFloat();
                        }else{
                            serial_input += c; // シリアルからの入力を連結
                        }
                    }
                    tx_message.data[0] = 'B';
                    tx_message.data[1] = 'A';
                    ((float*)&tx_message.data[2])[0] = serial_input.substring(0).toFloat();
                    tx_message.data_length_code = 2 + sizeof(float);
                    if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
                    break;
                default:
                    break;
            }
            break;
        case 'R': // モーターリセット
            tx_message.data[0] = 'R';
            tx_message.data_length_code = 1;
            if (twai_transmit(&tx_message, pdMS_TO_TICKS(100)) == ESP_OK);
            break;
        default:
            return;
    }
}

// safe ratio: |a| / (|a| + |b|), returns 0 when denominator is ~0 to avoid NaN
static inline float safe_share(float a, float b){
    float da = fabsf(a);
    float db = fabsf(b);
    float denom = da + db;
    if(denom <= 1e-6f) return 0.0f;
    return da / denom;
}

static inline float clamp_torque(float t, float limit){
    if(std::isnan(t)) return 0.0f;
    if(t > limit) return limit;
    if(t < -limit) return -limit;
    return t;
}