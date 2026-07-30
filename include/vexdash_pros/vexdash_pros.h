#pragma once

// vexdash_pros -- the ONE header a PROS user includes. Provides a
// one-line-init façade over transport + Session + ConnectionPump + Task.
//
// CONTAINS PROS HEADERS (transitively): include this only from PROS project
// sources, never in the host build.
//
// Typical usage (see examples/pros-pid-tuning):
//
//   #include "vexdash_pros/vexdash_pros.h"
//
//   void register_all(vexdash::Session& s, void*) {
//     // (re)declare channels/configs/commands/devices here. Called once at
//     // startup and again after every reconnect, so keep it idempotent
//     // (re-declaring reuses the same ids in declaration order).
//   }
//
//   void initialize() {
//     vexdash::init_usb(register_all);   // <-- one line, USB path
//     // ...or for the Smart Port path:
//     // vexdash::init_smartport(20, 921600, register_all);
//   }
//
//   void opcontrol() {
//     while (true) {
//       vexdash::telemetry().put(err_ch, error);  // buffer samples
//       pros::delay(10);                           // pump flushes at cadence
//     }
//   }
//
// The façade owns everything with static storage (no heap). init_* is
// idempotent-safe to call once; calling twice is a no-op after the first.

#include <cstdint>

#include "vexdash/protocol_types.h"  // DeviceType for declare_device()
#include "vexdash/session.h"
#include "vexdash_pros/command_registry.h"  // CommandHandler, for declare_command()
#include "vexdash_pros/connection_pump.h"

// 前向宣告，讓 watch_motor() 能在標頭吃 pros::Motor& 而不強迫 host 端 include PROS
// header。真正的型別定義由使用者的 PROS 專案（main.h / api.h）在 .cpp 端提供。
namespace pros {
inline namespace v5 {
class Motor;
}
}  // namespace pros

namespace vexdash {

// Starts vexdash over the Brain micro-USB serial link (MVP path). Spawns
// the background pump task. `on_register` (default nullptr) is invoked to
// (re)declare your channels/configs/commands/devices at startup and on every
// reconnect. Leave it nullptr (the common case) and the façade installs a
// built-in callback that walks the watch() registry -- you never write a
// register_all. Pass your own only for advanced/manual registration.
// `show_status` (default true) enables the on-Brain status HUD -- IMPORTANT
// because UsbSerialTransport takes over stdout for the vexdash data channel,
// so printf/std::cout can no longer be used to debug (it would corrupt the
// stream). The Brain screen is your debug window. `cfg` tunes rates/timeouts
// (defaults 50Hz telemetry, 1Hz heartbeat). Returns a reference to the live
// Session (also reachable via session()).
Session& init_usb(RegisterCallback on_register = nullptr, bool show_status = true,
                  void* user_data = nullptr, const PumpConfig& cfg = PumpConfig{});

// Starts vexdash over a Smart Port generic-serial link (for the
// ESP32/RS-485 bridge or a USB-RS485 dongle). `smart_port` is 1..21.
// `on_register` default nullptr -> built-in watch() registry callback (same
// as init_usb). `show_status` enables the on-Brain HUD (default false here:
// the Smart Port path does NOT commandeer stdout, so printf debugging still
// works and the HUD is optional).
Session& init_smartport(std::uint8_t smart_port, std::int32_t baudrate,
                        RegisterCallback on_register = nullptr, bool show_status = false,
                        void* user_data = nullptr, const PumpConfig& cfg = PumpConfig{});

// 一鍵起步：USB 直連 + 合理預設（50Hz 遙測、1Hz 心跳、自動重連重播、Brain 狀態
// HUD 開）。等同 init_usb() 不帶參數，只是名字更直白。搭配 watch()/watch_config()
// 使用：先登記變數，再 quick_start()。
inline Session& quick_start() { return init_usb(); }

// ---- 方案 A：watch 一行流登記（使用者主要用這幾個）----------------------
//
// 心智模型只有一件事：「給名字＋給變數位址」。背景 pump 會自動把 watch 的變數上報
// 成圖表、把 watch_config 的變數做成 dashboard 可調參數並自動回寫——使用者不用寫
// register_all、不用寫 callback、不用寫 memcpy、迴圈裡也不用寫任何 put()。
//
// 用法：在 initialize() 裡「先」登記、「再」quick_start()（登記要早於背景 task 起跑）。
//   watch_config("kP", &kP);   // 可在 dashboard 上調（雙向）
//   watch("error", &error);    // 唯讀，自動畫進圖表
//   quick_start();
//
// 只可登記全域 / static / 生命週期夠長的變數（同現況 callback 抓 &g_kp 的約束）。
// 型別支援 double / float / int32_t / bool，由多載自動推導，使用者不碰 ValueType。
// 上限與超額行為：登記表固定 64 項（watch_motor 一次佔 4 項）；超過上限、名字無效
// （空字串／超過 kMaxNameLen）或傳入 nullptr 的登記會失敗，透過下方的 bool 回傳值
// 告知（見 2026-07-19 前置修繕 FIX-2）——**呼叫端可以忽略回傳值**（如 quick-start
// 範例那樣直接當一行敘述寫，向後相容零改動），但想在啟動時抓出「表滿/打錯名字」
// 這類無聲失敗，就檢查回傳值。回傳值只是把 WatchRegistry::add() 本來就有的 bool
// 透傳出來，不是新行為，只是不再丟棄它。詳見 docs/quick-start.zh-TW.md 的「常見雷」。

// 唯讀上報：登記一個變數，背景自動取樣並畫成圖表線。`unit` 是選填單位字串（如
// "deg"、"rpm"）；`device_port` 選填（1-21 智慧埠），預設不歸屬任何埠；`path` 選填
// 機構／群組路徑（如 "drive/pid"），**與 watch_config 的 `group` 是同一個命名空間**
// ——填同一個字串，dashboard 的「調校焦點」就會把這條圖表線跟那個機構的可調參數放
// 在同一組。不填＝維持原樣（wire 上一個 byte 都不多，前端退回用名字猜分組；名字猜
// 分組會被孔位名撞到，這正是填 path 要解決的問題）。
//
//   watch_config("kP", &kP, "drive/pid");            // 參數屬於底盤 PID
//   watch("error", &error, "rpm", -1, "drive/pid");  // 這條圖表線也是（明講，不用猜）
//
// 回傳 true＝已登記／已覆蓋既有同名項；false＝表滿（64 項已滿）、名字無效（空字串
// 或超過 63 bytes）、或 value 為 nullptr。
bool watch(const char* name, double* value, const char* unit = "", int device_port = -1,
           const char* path = "");
bool watch(const char* name, float* value, const char* unit = "", int device_port = -1,
           const char* path = "");
bool watch(const char* name, std::int32_t* value, const char* unit = "", int device_port = -1,
           const char* path = "");
bool watch(const char* name, bool* value, const char* unit = "", int device_port = -1,
           const char* path = "");

// 可調雙向：登記一個變數當 dashboard 可調參數；使用者在網頁改值，背景自動回寫進
// 這個變數。`group` 選填 UI 群組路徑（如 "drive/pid"），預設放根層。
// 回傳值意義同 watch()。
bool watch_config(const char* name, double* value, const char* group = "");
bool watch_config(const char* name, float* value, const char* group = "");
bool watch_config(const char* name, std::int32_t* value, const char* group = "");
bool watch_config(const char* name, bool* value, const char* group = "");

// 物件助手：一次登記一顆馬達的常用遙測（位置 deg / 轉速 rpm / 溫度 °C / 電流 mA），
// 頻道名為 "<name>.pos" 等，並自動帶上馬達的 device_port。`path` 選填（意義同 watch()），
// 四條頻道會一起掛進該機構分組——底盤四顆馬達全填 "drive" 就一次分好組。
// `motor` 必須在程式執行期
// 全程存活（全域 / static）。[NEEDS-HW-VERIFICATION]：需 pros::Motor，僅能實機驗。
// 回傳 true＝四條 channel 全部登記成功；false＝至少一條失敗（通常是表滿——4 項一起
// 登記，若剩餘槽位不足 4 個，先登記的仍會成功、只有超出容量的那幾條失敗，回傳值
// 只告知「整體是否全數成功」，個別哪一條失敗需檢查 dashboard 頻道清單）。
bool watch_motor(const char* name, pros::Motor& motor, const char* path = "");

// ---- 自訂按鈕（CMD_DEF/COMMAND，盤點-dashboard功能稽核-2026-07-27.md 前提 2/C6）---
//
// 跟 watch()/watch_config() 同一套心智模型：「給名字、給一個處理函式」。這是盤點報告
// 點名的缺口——watch 系列都有一行式門面，declare_command 原本沒有，dashboard 的
// 「自訂按鈕 Commands」面板因此在教學現場幾乎不會亮。用法（在 quick_start() 之前登記，
// 跟 watch()/watch_config() 同順序要求）：
//
//   void on_reset() { odom_x = 0; odom_y = 0; }
//   declare_command("Reset Odometry", &on_reset, /*requires_confirm=*/true);
//   quick_start();
//
// 背景會自動送出 CMD_DEF、按鈕觸發時自動呼叫你的函式、重連時自動重新宣告（同一顆
// 按鈕、同一個 id，見 command_registry.h 的 idempotent 說明）。回傳 true＝已登記／
// 已覆蓋既有同名項；false＝表滿（16 項）、名字無效、或 handler 為 nullptr。
//
// 本一行式門面只涵蓋「零參數觸發鈕」（教學現場最常見的形狀）；需要帶參數的指令
// （例如「移動到 (x,y)」）仍走底層 API：session().command().declare(name,
// CmdParamSpec[], count, requires_confirm) + set_callback()，兩者可並存。
bool declare_command(const char* name, CommandHandler handler, bool requires_confirm = false,
                     void* user_data = nullptr);

// ---- 場地 Field（FIELD_OPS SET_POSE，盤點-dashboard功能稽核-2026-07-27.md 前提 1）---
//
// 一行式：在 opcontrol()/里程計更新迴圈裡跟 watch() 上報一樣頻繁呼叫即可，dashboard
// 的「場地 Field」面板會即時畫出機器人位置＋朝向＋走過的軌跡殘影。這是盤點報告點名
// 的缺口——協定/lib-core 的 FieldView::set_pose() 早就有，但門面沒包裝、範例沒示範，
// 所以車端不主動呼叫就是一格永遠空白的格線。等同 `session().field().set_pose(...)`
// 的薄轉發（省一次 `session().field()` 中繼查找）。回傳值同 FieldView::set_pose()
// （false＝底層 transport 寫入失敗，不是常見情況，可忽略回傳值）。
bool set_pose(double x_mm, double y_mm, double heading_rad);

// ---- 裝置宣告 + 自動掃描（DEVICE_MAP，票 WS9）---------------------------
//
// 比賽場景要看的是：「我程式裡用到的裝置，插好了沒、型別對不對」。所以先用一行一
// 個「宣告」你會用到的裝置（帶名字），背景 pump 再自動掃描去填每個宣告埠的實際狀態：
//
//   void initialize() {
//     vexdash::declare_device(3,  DeviceType::kMotor,    "left_drive");
//     vexdash::declare_device(10, DeviceType::kImu,      "imu");
//     vexdash::declare_device(vexdash::adi_port('A'), DeviceType::kOptical, "line");
//     vexdash::quick_start();   // 先宣告、再啟動（跟 watch() 一樣的順序）
//   }
//
// 行為：
//   * 宣告過的裝置一律送給 dashboard（就算沒插到也送，connected=false）——這樣「宣告
//     了卻沒插好／插錯型別」會在埠孔位圖上顯示成異常，正是比賽前檢查接線要的。
//   * 掃到但沒宣告的裝置也會送，但「名字留空」讓前端能區分（意外插上的裝置）。
//   * 從頭到尾沒宣告又沒插的埠不送（面板只顯示你在意的）。
//   * 開機送一次、之後每 ~1 秒偵測插拔，只在內容變更時重送（不洗高頻路徑）。
//
// `port` 用協定埠號：1~21 智慧埠、22~29 ADI（可用 adi_port('A')）。回傳 false＝埠越界
// （非 1~29）、名字超過 63 bytes、或宣告表已滿（29 項）。學生手寫一行一個即可；日後
// dplib/接線工作室會自動產生這些宣告（後續票）。
//
// 預設「開」。要完全手動改用 device_map().add_port(...)（進階/傳自訂 on_register）時，
// 在 quick_start()/init_* 之「前」呼叫 enable_device_scan(false) 關掉自動掃描（一行開
// 關）；傳自訂 on_register 時本功能本來就不介入，手動 add_port() 用法相容保留。
//
// ADI（三線 A~H）限制：類比/數位訊號沒有裝置身分，物理上無法偵測，宣告的 ADI 裝置
// 只能「照宣告當作在」（connected=true，未實際驗證），不會假裝偵測得到插拔。
bool declare_device(std::uint8_t port, DeviceType type, const char* name = "");
void enable_device_scan(bool on);

// Accessors for the singletons created by init_*. Undefined behavior if
// called before a successful init_*.
Session& session();
Telemetry& telemetry();      // shorthand for session().telemetry()
Config& config();            // shorthand for session().config()
Command& command();          // shorthand for session().command()
DeviceMap& device_map();     // shorthand for session().device_map()
DeviceStatus& device_status();  // v1.3 shorthand for session().device_status() (§5.13)
ConnectionPump& pump();

// True once an init_* has run.
bool is_initialized();

}  // namespace vexdash
