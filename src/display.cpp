#include "display.hpp"

M5Canvas canvas(&M5.Display);

void display_clear() {
    M5.begin();
    M5.Display.startWrite();
    M5.Display.setRotation(1); // ディスプレイの回転設定
    M5.Display.fillScreen(TFT_BLACK); // 画面を黒で塗りつぶす
    M5.Display.setTextColor(TFT_WHITE); // テキストの色を白
    M5.Display.setTextSize(2); // テキストサイズの設定
    M5.Display.setCursor(0, 0); // カーソル位置の設定
    M5.Display.endWrite();
}

size_t display_print(const char *str) {
    M5.Display.startWrite();
    size_t len = M5.Display.print(str);
    M5.Display.endWrite();
    return len;
}

size_t display_println(const char *str) {
    M5.Display.startWrite();
    size_t len = M5.Display.println(str);
    M5.Display.endWrite();
    return len;
}

size_t display_printCenter(const char *str) {
    M5.Display.startWrite();
    M5.Display.setTextDatum(MC_DATUM); // 中央揃え
    M5.Display.setCursor(M5.Display.width() / 2, M5.Display.height() / 2); // 中央にカーソルを設定
    size_t len = M5.Display.print(str);
    M5.Display.setTextDatum(TL_DATUM); // 元の左上揃えに戻す
    M5.Display.endWrite();
    return len;
}

void display_menu() {
    display_clear();
    display_print("M5Stack Control Menu   ");
    M5.Display.startWrite();
    M5.Display.println(M5.Power.getBatteryLevel());
    M5.Display.endWrite();
    display_println("---------------------");
    display_println("1. Sine Wave");
    display_println("2. IMU Mode");
    display_println("3. IMU Mode 2");
    display_println("4. pace_FixCycle Mode");
    display_println("5. trot_FixCycle Mode");
    display_println("6. pace_VPB Mode");
    display_println("7. trot_VPB Mode");
    display_println("8. pace_VPB_rev Mode");
    display_println("---------------------");
    display_print("Select an option:");
}