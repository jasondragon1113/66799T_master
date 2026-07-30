#pragma once

#include <cstddef>
#include <cstdint>

#include "vexdash/command.h"
#include "vexdash/session.h"

// CommandRegistry -- 方案 A 對「自訂按鈕」的一行流登記，跟 WatchRegistry
// （watch_registry.h）同一套心智模型：「給名字、給一個處理函式」。
//
// 這是盤點報告點名的缺口（盤點-dashboard功能稽核-2026-07-27.md 前提 2／C6／四-3）：
// watch()/watch_config()/watch_motor() 都有好用的一行式門面，declare_command 原本
// 沒有——`examples/pros-pid-tuning` 唯一提到 `s.command().declare(...)` 的地方是寫在
// 註解裡，從沒有真的呼叫過。Commands 面板在教學現場因此幾乎不會亮，門檻比其他面板
// 高一階。這個檔案把它補齊到跟其他門面同一個難度。
//
// 使用者心智模型（門面 vexdash_pros.h 的 declare_command()）：
//
//   void on_reset() { odom_x = 0; odom_y = 0; }
//   vexdash::declare_command("Reset Odometry", &on_reset, /*requires_confirm=*/true);
//   vexdash::quick_start();
//
// 三件事自動化（比照 WatchRegistry 的三件事，這裡對應到 Command 而非 telemetry/config）：
//   1. 自動註冊（含重連重播）：declare_all() 走訪登記表逐一呼叫 Command::declare()。
//      lib-core 的 Command::declare() 本身依名字去重（同名重宣告會重用既有 id、重送
//      CMD_DEF、保留既有 callback——2026-07-08 WORKLOG 的冪等修正），所以 declare_all()
//      天生 idempotent，滿足 ConnectionPump 的重播契約，跟 WatchRegistry 完全同一套道理。
//   2. 自動接上 callback：declare_all() 每次都呼叫 Command::set_callback()，把收到的
//      COMMAND 轉呼叫使用者的 CommandHandler；重複設定同一個 handler 是安全的
//      no-op（同一個 Entry 位址、同一個函式指標）。
//   3. 使用者不用碰 CmdParamValue／dispatch_command——本版本只涵蓋「零參數觸發鈕」，
//      也就是教學現場最常見的形狀（「一鍵跑 auton」「重置里程計」，見範例）。需要帶
//      參數的指令仍走底層 API：Session::command().declare(name, CmdParamSpec[], count,
//      requires_confirm) + set_callback()——跟 watch() 的「進階用法退回底層 API」是
//      同一個設計取捨，未改動、仍相容。
//
// 這個類刻意「不含任何 PROS header」（只依賴平台無關的 lib-core），因此納入 host build
// 做單元測試（比照 watch_registry.h/connection_pump.h 的隔離原則）。
//
// English summary: a one-line façade for "custom action buttons" mirroring
// WatchRegistry's ergonomics -- give a name and a zero-arg handler function.
// declare_all() is idempotent (Command::declare() dedups by name) so it can
// be replayed on every reconnect exactly like the telemetry/config registry.
// Parameterized commands still go through the lower-level Session::command()
// API, unchanged.

namespace vexdash {

// 零參數指令處理函式（觸發鈕按下時呼叫一次）。user_data 為登記時傳入的 context。
using CommandHandler = void (*)(void* user_data);

class CommandRegistry {
 public:
  // 固定容量（無 heap）。教學現場的自訂按鈕數量遠低於 lib-core 的 kMaxCommands(32)，
  // 16 個對「一鍵跑 auton／重置里程計／…」這類用途綽綽有餘。命名故意不叫
  // kMaxCommands（會跟 lib-core 的 vexdash::kMaxCommands 同名易誤讀成同一個常數
  // ——兩者其實是完全獨立的容量，2026-07-27 審查回饋，比照 kMaxWatches 前例）。
  static constexpr std::size_t kMaxRegisteredCommands = 16;

  CommandRegistry() = default;

  // 登記一個零參數觸發鈕。同名再次登記＝覆蓋（更新 handler/requires_confirm/
  // user_data，declare_all() 重新對齊時沿用同一個 lib-core CommandId）。回傳
  // false＝表滿（kMaxRegisteredCommands）、名字無效（空字串或超過 kMaxNameLen）、或
  // handler 為 nullptr。
  bool add(const char* name, CommandHandler handler, bool requires_confirm, void* user_data);

  // 走訪登記表逐一呼叫 Command::declare() + set_callback()。天生 idempotent
  // （Command::declare 依名字去重），可安全在啟動、每次重連、每次週期自癒重送時
  // 重複呼叫，跟 WatchRegistry::declare_all 同一份契約。
  void declare_all(Session& session);

  std::size_t size() const { return count_; }
  void clear() { count_ = 0; }  // Test-only; not called in normal use. 中文：測試用

 private:
  struct Entry {
    char name[kMaxNameLen + 1] = {0};
    bool requires_confirm = false;
    CommandHandler handler = nullptr;
    void* user_data = nullptr;
  };

  // 轉接：lib-core 的 CommandCallback 簽名是 (id, values, count, user_data)；本
  // 版本只涵蓋零參數指令，把 values/count 丟棄，呼叫 entry->handler(entry->user_data)。
  static void dispatch(CommandId id, const CmdParamValue* values, std::size_t value_count, void* user_data);

  Entry* find(const char* name);

  Entry entries_[kMaxRegisteredCommands];
  std::size_t count_ = 0;
};

}  // namespace vexdash
