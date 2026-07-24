#pragma once

#include <cstddef>
#include <cstdint>

#include "vexdash/config.h"
#include "vexdash/protocol_types.h"
#include "vexdash/session.h"
#include "vexdash/telemetry.h"

// WatchRegistry -- 方案 A（watch 一行流登記）的核心登記表。
//
// 使用者的心智模型只有一件事：「給個名字、給個變數位址」。登記表把三件雜事全部
// 自動化，使用者不必再理解 register_all / memcpy callback / 每圈 put / ChannelOptions：
//
//   1. 自動註冊（含重連重播）：declare_all() 走訪登記表逐一 declare_*。因為 lib-core
//      的 declare_* 依名字去重（同名重宣告會重用既有 id），declare_all() 本身天生
//      idempotent，正好滿足 ConnectionPump 的重播契約（重連/週期自癒重送都直接沿用，
//      協定與 dashboard 前端零改動）。
//   2. 自動上報：sample_all() 讀出每個 telemetry 登記項的指標值並 put() 進 telemetry。
//      門面把它掛到 pump 的 flush 前取樣點（PumpConfig::pre_flush），使用者迴圈裡一行
//      put 都不用寫。
//   3. 自動回寫：config 登記項共用一個回寫 callback，收到 CONFIG_SET 就把值 memcpy
//      回指標——使用者不寫 callback、不寫 memcpy。
//
// 這個類刻意「不含任何 PROS header」（只依賴平台無關的 lib-core），因此納入 host build
// 做單元測試（比照 connection_pump 的隔離原則）。需要 pros::Motor 之類物件的助手
// （watch_motor）放在門面 vexdash_pros.cpp，透過 add_fn() 登記取樣函式。
//
// 限制與風險（與提案結論一致，僅需註解說明，非新增風險）：
//   - 撕裂讀：sample_all() 在 pump task 讀指標、使用者在 opcontrol 寫指標；f64/i32 在 V5
//     上非原子，理論上可能讀到撕裂值。但這只是顯示用途、撕裂一個樣本無害，且與現況
//     config callback 的併發模型完全相同（回寫也在 pump task 執行）——沒有引入新風險。
//   - 懸掛指標：只可 watch 全域 / static / 生命週期夠長的變數（與現況 callback 抓 &g_kp
//     同樣的約束）。
//   - 表容量固定（kMaxWatches），超過就登記失敗（回傳 false），比照 lib-core 的
//     kMaxChannels/kMaxConfigParams。

namespace vexdash {

// 指標所指的 C 型別（決定怎麼讀寫指標；wire 上的 ValueType 由此推導）。
enum class WatchScalar : std::uint8_t {
  kF64,   // double*   -> wire kF64
  kF32,   // float*    -> wire kF64 (widened to double on the wire). 中文：float 在線上升為 double
  kI32,   // int32_t*  -> wire kI32
  kBool,  // bool*     -> wire kBool
};

class WatchRegistry {
 public:
  // 固定容量（無 heap）。比照 lib-core 的 kMaxChannels / kMaxConfigParams。
  static constexpr std::size_t kMaxWatches = 64;

  WatchRegistry() = default;

  // ---- 登記：telemetry（唯讀上報，指標式）--------------------------------
  // 回傳 true＝已登記/已覆蓋；false＝表滿、名字無效、或 ptr 為 nullptr（null 指標
  // 在入口直接拒絕，不會進表——與 sample_all/declare_all 的防護一致）。同名同類
  // 後者覆蓋前者（更新指標與屬性；覆蓋後既有的 id 快取會在下次 declare_all()
  // 重新對齊）。表滿（kMaxWatches=64）時登記靜默失敗（回 false），門面的
  // watch()/watch_config() 不回傳值，超額項會被靜默忽略——上限與行為見
  // docs/quick-start.zh-TW.md 的「常見雷」。
  bool add(const char* name, double* ptr, const char* unit = "", int device_port = -1);
  bool add(const char* name, float* ptr, const char* unit = "", int device_port = -1);
  bool add(const char* name, std::int32_t* ptr, const char* unit = "", int device_port = -1);
  bool add(const char* name, bool* ptr, const char* unit = "", int device_port = -1);

  // ---- 登記：telemetry（取樣函式式，供 watch_motor 等物件助手用）----------
  // sampler(obj) 每次回傳一個 double 樣本（wire 型別固定 kF64）。obj 是傳給
  // sampler 的 context（例如 pros::Motor*）。
  bool add_fn(const char* name, double (*sampler)(void* obj), void* obj, const char* unit = "",
              int device_port = -1);

  // ---- 登記：config（可調雙向，指標式）-----------------------------------
  // group 是 dashboard 上的 UI 群組路徑（protocol.md §5.6），"" = 根。
  bool add_config(const char* name, double* ptr, const char* group = "");
  bool add_config(const char* name, float* ptr, const char* group = "");
  bool add_config(const char* name, std::int32_t* ptr, const char* group = "");
  bool add_config(const char* name, bool* ptr, const char* group = "");

  // 走訪登記表逐一 declare_*（telemetry -> declare_channel_ex，config ->
  // declare_f64/i32/bool + set_callback）。天生 idempotent：可安全在啟動、每次
  // 重連、每次週期自癒重送時重複呼叫（正是 pump 的 on_register 契約要的）。
  void declare_all(Session& session);

  // flush 前取樣：把每個 telemetry 登記項的目前值讀出並 put() 進 telemetry。
  // 尚未 declare_all()（或 declare 失敗）的項目其 channel id 無效，會被跳過。
  void sample_all(Telemetry& telemetry);

  std::size_t size() const { return count_; }
  void clear() { count_ = 0; }  // Test-only; not called in normal use (the registry lives as long as the program). 中文：測試用；正常使用不呼叫（登記表生命週期同程式）

 private:
  enum class Kind : std::uint8_t { kTelemetry, kConfig };

  struct Entry {
    char name[kMaxNameLen + 1] = {0};
    // config: UI 群組路徑；telemetry: 單位字串。兩者互斥（依 kind），共用一個緩衝。
    char group_or_unit[kMaxNameLen + 1] = {0};
    Kind kind = Kind::kTelemetry;
    WatchScalar scalar = WatchScalar::kF64;
    bool is_fn = false;                        // telemetry, sampler-function form. 中文：telemetry 取樣函式式
    void* ptr = nullptr;                       // Variable address; the sampler's obj when is_fn. 中文：變數位址；is_fn 時為 sampler 的 obj
    double (*sampler)(void* obj) = nullptr;    // Sampler when is_fn, otherwise nullptr. 中文：is_fn 時的取樣器，否則 nullptr
    int device_port = -1;                      // telemetry only; -1 = no device_port. 中文：telemetry 專用；-1 = 不帶 device_port
    ChannelId ch_id = 0xFFFF;                  // Cached after declare_all (starts at kInvalidChannelId). 中文：declare_all 後快取（kInvalidChannelId 起始）
    ConfigId cfg_id = 0xFFFF;                  // Same as above. 中文：同上
  };

  // 共用 config 回寫 callback：user_data = 指向該 Entry。收到 CONFIG_SET 就把
  // value_bytes 依 scalar 寫回 entry->ptr。
  static void on_config_set(ConfigId id, const std::uint8_t* value_bytes, void* user_data);

  // 依名字＋類別找既有項（同 lib-core：telemetry 名字空間與 config 名字空間獨立）。
  Entry* find(const char* name, Kind kind);
  // 找一個可用的槽（既有同名同類 -> 覆蓋；否則 append）。滿了回 nullptr。
  Entry* acquire(const char* name, Kind kind);

  Entry entries_[kMaxWatches];
  std::size_t count_ = 0;
};

}  // namespace vexdash
