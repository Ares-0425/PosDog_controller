# PosDog_controller
四脚歩行ロボット「PosDog」のコントローラーユニットのプログラム
## 動作環境
プログラムはVSCode＋PlatformIO環境で作成しています。
## ファイル
### プログラム
レポジトリ全体がPlatformIOのプログラムフォルダになっていますが、いじるべきプログラムは"src"フォルダ内に入っています。
- main.cpp  
  メインプログラムファイル。setupとloopの実行順設定。
- can.hpp  
  can.cppのヘッダー
- can.cpp  
  can通信設定。
- control.hpp  
  control.cppのヘッダー
- control.cpp  
  ロボット動作の細かい設定ファイル
- display.hpp  
  display.cppのヘッダー
- display.cpp  
  表示器関係ファイル
- imu.hpp  
  imu.cppのヘッダー
- imu.cpp  
  IMU（慣性系、LSM9DSV）センサ関係。センサーセットアップ、カルマンフィルタ、位相同期ループなど。
- sd.hpp  
  sd.cppのヘッダー
- sd.cpp
  SDカードの設定と書き込み準備、書き込みキューの作成、書き込みタスクの設定など
## 基本の使いかた
