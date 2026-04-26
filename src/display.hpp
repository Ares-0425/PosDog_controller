#ifndef DISPLAY_HPP
#define DISPLAY_HPP

#include <Arduino.h>
#include <M5Unified.h>

extern M5Canvas canvas;

// グローバル変数宣言

// グローバル関数宣言
void display_clear(void);
size_t display_print(const char *str);
size_t display_println(const char *str);
size_t display_printCenter(const char *str);
void display_menu(void);

#endif