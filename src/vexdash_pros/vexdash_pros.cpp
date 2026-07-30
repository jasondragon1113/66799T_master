// [NEEDS-HW-VERIFICATION] Compiled only inside a PROS project.

#include "vexdash_pros/vexdash_pros.h"

#include <cstdio>
#include <new>

#include "api.h"        // pros::Motor accessors for watch_motor()
#include "pros/apix.h"  // pros::c::registry_get_plugged_type for device scanning
#include "vexdash_pros/device_scanner.h"
#include "vexdash_pros/pros_task.h"
#include "vexdash_pros/smartport_transport.h"
#include "vexdash_pros/status_screen.h"
#include "vexdash_pros/usb_serial_transport.h"
#include "vexdash_pros/watch_registry.h"

namespace vexdash {
namespace {

// 方案 A：全域 watch 登記表（固定 static 儲存，無 heap）。watch()/watch_config()/
// watch_motor() 往這張表登記；pump 的預設 on_register 走訪它做註冊、pre_flush 鉤子
// 走訪它做上報取樣。
WatchRegistry g_registry;

// 方案 A：全域自訂按鈕登記表（同上，無 heap）。declare_command() 往這張表登記；
// 跟 g_registry 一樣由 default_register() 走訪做（重連也會走的）idempotent 註冊。
CommandRegistry g_command_registry;

// 方案 A 的裝置掃描器（DEVICE_MAP 自動化，票 WS9）。掃 21 個智慧埠、記快照、偵測
// 插拔——純邏輯在 host 可測的 DeviceScanner，這裡只補上唯一碰 PROS 的讀埠函式。
DeviceScanner g_device_scanner;

// 學生零設定就會動：預設自動啟用埠掃描。一行開關＝在 quick_start() 前呼叫
// enable_device_scan(false) 關掉（見標頭）。
bool g_device_scan_enabled = true;

// 唯一碰 PROS 的讀埠接縫（薄到一行）：回傳 pros::c::v5_device_e_t（DPLIB
// pros/apix.h:660，埠 0~20 零起算）。host build 不含本檔，所以 DeviceScanner 的
// 純邏輯照樣能在 host 測；上機時才由這個真實讀取器餵資料。
int read_plugged(std::uint8_t pros_port_0based, void*) {
  return static_cast<int>(pros::c::registry_get_plugged_type(pros_port_0based));
}

// DEVICE_STATUS (0x0E, protocol.md §5.13, 票 WS10-A) 的唯一碰 PROS 的值讀取接縫。
// 依「實測型別」讀該型別的標準值集，寫進 out[]、回傳寫入幾個（順序＝§5.13 契約）。
// 簽名不含 PROS 型別，故 DeviceScanner::send_status 的調度邏輯在 host 可測；本函式
// 只在 PROS 專案內編譯（host build 不含本檔）。每個 getter 的 header 出處與單位：
//   MOTOR    temperature = pros/motors.h:790 motor_get_temperature       -> °C   (double)
//            power       = pros/motors.h:763 motor_get_power             -> W    (double)
//            current     = pros/motors.h:423 motor_get_current_draw      -> mA   (int32)
//            rpm         = pros/motors.h:396 motor_get_actual_velocity   -> RPM  (double)  [WS10-F]
//   ROTATION angle(絕對) = pros/rotation.h:247 rotation_get_angle    -> centideg 0..36000 (int32)
//            position(相對/累計) = pros/rotation.h:193 rotation_get_position -> centideg (int32)
//   DISTANCE distance    = pros/distance.h:64  distance_get          -> mm   (int32)
//            confidence  = pros/distance.h:94  distance_get_confidence-> 0..63 (int32)
//   OPTICAL  hue         = pros/optical.h:130 optical_get_hue        -> 0..359.999 deg (double)
//            proximity   = pros/optical.h:220 optical_get_proximity  -> 0..255 (int32)
//            brightness  = pros/optical.h:190 optical_get_brightness -> 0..1.0 (double)
//   IMU      heading     = pros/imu.h:276 imu_get_heading -> [0,360) deg (double)
//            pitch       = pros/imu.h:478 imu_get_pitch   -> (-180,180) deg (double)
//            roll        = pros/imu.h:512 imu_get_roll    -> (-180,180) deg (double)
//   BATTERY  voltage     = pros/misc.h:733 battery_get_voltage     -> mV  (int32; VEXos native)
//            current     = pros/misc.h:751 battery_get_current     -> mA  (int32; VEXos native)
//            capacity    = pros/misc.h:787 battery_get_capacity    -> %   (double, 0..100)
//            temperature = pros/misc.h:769 battery_get_temperature -> °C  (double)
//   （WS10-D 電池：非 smart port，wire port=0，永遠送；battery_get_* 的 header 註解未明寫單位，
//     單位由回傳型別 int32=milli-單位/double=%,°C ＋ VEXos 原生行為推定，與馬達 current 同用 mA 一致）
//   （WS10-F 馬達 rpm：motor_get_actual_velocity 的 header doxygen 明寫「回傳 RPM」，故直接引用，非推定。
//     可為負值＝實際轉向；「負埠號使回傳值變號」是給反轉安裝馬達的既有 PROS 慣例，與此處讀值邏輯無關，
//     車端一律照 motor_get_actual_velocity 原樣回傳，不額外處理正負。）
// 單位一律送 PROS 原生值（不換算），前端要顯示成 A/度/V 自行除；縮放取捨見 protocol.md §5.13。
std::uint8_t read_device_values(std::uint8_t wire_port, DeviceType observed_type, float* out,
                                std::uint8_t max_out, void*) {
  const std::uint8_t port = wire_port;  // §5.13 只送 smart port 1..21；PROS getter 亦用 1..21
  switch (observed_type) {
    case DeviceType::kBattery:
      // WS10-D: Brain-internal battery (no port arg; VEXos-global getters).
      // 中文：Brain 內建電池，getter 不吃埠號（全域）。
      if (max_out < 4) return 0;
      out[0] = static_cast<float>(pros::c::battery_get_voltage());      // mV
      out[1] = static_cast<float>(pros::c::battery_get_current());      // mA
      out[2] = static_cast<float>(pros::c::battery_get_capacity());     // %
      out[3] = static_cast<float>(pros::c::battery_get_temperature());  // °C
      return 4;
    case DeviceType::kMotor:
      // WS10-F: value set widened 3->4 (append rpm at the END, see
      // protocol_types.h device_status_value_count() comment on why appending
      // rather than inserting keeps this forward/backward compatible).
      if (max_out < 4) return 0;
      out[0] = static_cast<float>(pros::c::motor_get_temperature(static_cast<std::int8_t>(port)));
      out[1] = static_cast<float>(pros::c::motor_get_power(static_cast<std::int8_t>(port)));
      out[2] = static_cast<float>(pros::c::motor_get_current_draw(static_cast<std::int8_t>(port)));
      out[3] = static_cast<float>(pros::c::motor_get_actual_velocity(static_cast<std::int8_t>(port)));  // RPM
      return 4;
    case DeviceType::kRotation:
      if (max_out < 2) return 0;
      out[0] = static_cast<float>(pros::c::rotation_get_angle(port));     // 絕對 centideg
      out[1] = static_cast<float>(pros::c::rotation_get_position(port));  // 相對/累計 centideg
      return 2;
    case DeviceType::kDistance:
      if (max_out < 2) return 0;
      out[0] = static_cast<float>(pros::c::distance_get(port));             // mm
      out[1] = static_cast<float>(pros::c::distance_get_confidence(port));  // 0..63
      return 2;
    case DeviceType::kOptical:
      if (max_out < 3) return 0;
      out[0] = static_cast<float>(pros::c::optical_get_hue(port));         // 0..359.999
      out[1] = static_cast<float>(pros::c::optical_get_proximity(port));   // 0..255
      out[2] = static_cast<float>(pros::c::optical_get_brightness(port));  // 0..1.0
      return 3;
    case DeviceType::kImu:
      if (max_out < 3) return 0;
      out[0] = static_cast<float>(pros::c::imu_get_heading(port));  // [0,360)
      out[1] = static_cast<float>(pros::c::imu_get_pitch(port));    // (-180,180)
      out[2] = static_cast<float>(pros::c::imu_get_roll(port));     // (-180,180)
      return 3;
    default:
      return 0;  // no standard value set for this type -> not reported (§5.13)
  }
}

// declare_all()'s throttle hook: pros::delay(1) every kRegistrationYieldEvery
// declared frames, so the link-up registration burst gets a chance to drain
// the Smart Port TX FIFO mid-burst (second layer of defense alongside the
// transport-level bounded_retry_write -- see bounded_write.h and
// watch_registry.h's declare_all() doc comment).
// 中文：declare_all() 的節流鉤子：每 kRegistrationYieldEvery 幀 pros::delay(1)，
// 讓 link-up 註冊 burst 中途有機會排空 Smart Port TX FIFO（跟 transport 層的
// bounded_retry_write 是雙層保險，見 bounded_write.h 與 watch_registry.h 的
// declare_all() 註解）。
constexpr std::size_t kRegistrationYieldEvery = 5;
void registration_yield(void*) { pros::delay(1); }

// 內建的 idempotent 註冊回呼：on_register==nullptr 時 init_* 改用它（見 finish_init）。
// 除了走訪 watch 登記表宣告頻道外，順手掃一次埠、把當前埠地圖「無條件」整批送出——
// 開機/重連/週期自癒都會經過這裡，無條件重送＝DEVICE_MAP 的掉幀自癒（跟 CHANNEL_DEF
// 同款週期重送策略），確保初始快照就算掉一幀也會在下個自癒週期補回。
void default_register(Session& s, void*) {
  g_registry.declare_all(s, &registration_yield, nullptr, kRegistrationYieldEvery);
  g_command_registry.declare_all(s);
  if (g_device_scan_enabled) {
    g_device_scanner.scan(&read_plugged, nullptr);
    g_device_scanner.send(s.device_map());
  }
}

// 內建的 flush 前取樣回呼：一律掛到 pump 的 pre_flush（登記表為空時為 no-op）。
void default_sample(Session& s, void*) { g_registry.sample_all(s.telemetry()); }

// 週期性掃埠回呼（掛到 pump 的 device_scan 鉤子，~1s 一次）：
//  1) DEVICE_MAP（0x0D）只在插拔變化時重送，不是每 tick 洗訊息（掉幀自癒交給
//     default_register 的週期重送）。
//  2) DEVICE_STATUS（0x0E, §5.13, WS10-A）則**每個週期都送**——即時值持續變動，
//     不像埠地圖只在變更時送。同一個 ~1Hz 鉤子驅動，不另開 task。先 scan() 讓實測
//     快照最新，再用實測型別讀值送出。[NEEDS-HW-VERIFICATION]：read_device_values
//     的 PROS getter 需上機驗。
void default_device_scan(Session& s, void*) {
  const bool plug_changed = g_device_scanner.scan(&read_plugged, nullptr);
  if (plug_changed) {
    g_device_scanner.send(s.device_map());
  }
  // with_battery=true：電池永遠送（wire port 0，不靠宣告），走同一個讀值接縫（WS10-D）。
  g_device_scanner.send_status(s.device_status(), &read_device_values, nullptr, /*with_battery=*/true);
}

// watch_motor 的取樣器：void* obj = pros::Motor*，回傳一個 double 樣本。
double motor_pos(void* m) { return static_cast<pros::Motor*>(m)->get_position(); }
double motor_rpm(void* m) { return static_cast<pros::Motor*>(m)->get_actual_velocity(); }
double motor_temp(void* m) { return static_cast<pros::Motor*>(m)->get_temperature(); }
double motor_amp(void* m) {
  return static_cast<double>(static_cast<pros::Motor*>(m)->get_current_draw());
}

// Static storage for the singletons. We use aligned buffers + placement new
// so the façade owns everything with static-duration storage and no heap,
// while still supporting either transport type chosen at init time.
alignas(UsbSerialTransport) unsigned char g_usb_storage[sizeof(UsbSerialTransport)];
alignas(ProsSmartPortTransport) unsigned char g_sp_storage[sizeof(ProsSmartPortTransport)];
ITransport* g_transport = nullptr;

alignas(Session) unsigned char g_session_storage[sizeof(Session)];
Session* g_session = nullptr;

alignas(ConnectionPump) unsigned char g_pump_storage[sizeof(ConnectionPump)];
ConnectionPump* g_pump = nullptr;

alignas(StatusScreen) unsigned char g_screen_storage[sizeof(StatusScreen)];
StatusScreen* g_screen = nullptr;

alignas(ProsTask) unsigned char g_task_storage[sizeof(ProsTask)];
ProsTask* g_task = nullptr;

bool g_initialized = false;

void finish_init(RegisterCallback on_register, bool show_status, void* user_data,
                 PumpConfig cfg) {
  g_session = new (g_session_storage) Session(*g_transport, Role::kRobot);
  // on_register 為 nullptr（常見情況）時，改用內建走訪 watch 登記表的預設回呼——
  // 它天生 idempotent（declare_* 依名字去重），正好滿足 pump 的重連重播/週期自癒
  // 重送契約，使用者永遠不用寫 register_all。進階使用者仍可傳自訂 on_register。
  RegisterCallback reg = (on_register != nullptr) ? on_register : &default_register;
  // 一律掛上 flush 前取樣鉤子，讓 watch() 的變數自動上報。登記表為空時是 no-op，
  // 所以即使使用者傳了自訂 on_register 也不會有副作用（完全向後相容）。
  cfg.pre_flush = &default_sample;
  cfg.pre_flush_user_data = nullptr;
  // 只有走內建註冊路徑（on_register==nullptr）且未被關閉時，才掛週期掃埠鉤子。
  // 使用者若傳自訂 on_register（例如沿用手動 s.device_map().add_port(...)），我們
  // 完全不介入 DEVICE_MAP——避免自動掃描覆蓋掉他手動命名的埠（相容保留手動用法）。
  const bool builtin_register = (on_register == nullptr);
  if (builtin_register && g_device_scan_enabled) {
    cfg.device_scan = &default_device_scan;
    cfg.device_scan_user_data = nullptr;
  }
  g_pump = new (g_pump_storage) ConnectionPump(*g_session, cfg, reg, user_data);
  if (show_status) {
    g_screen = new (g_screen_storage) StatusScreen(*g_pump);
    // LLEMU must be initialized from the caller's context (initialize()),
    // BEFORE ProsTask spawns: pros::lcd::initialize() from a background task
    // deadlocks that task (observed on hardware, WORKLOG 2026-07-08).
    g_screen->init();
  }
  g_task = new (g_task_storage) ProsTask(*g_pump, g_screen);
  g_initialized = true;
}

}  // namespace

Session& init_usb(RegisterCallback on_register, bool show_status, void* user_data,
                  const PumpConfig& cfg) {
  if (!g_initialized) {
    g_transport = new (g_usb_storage) UsbSerialTransport();
    finish_init(on_register, show_status, user_data, cfg);
  }
  return *g_session;
}

Session& init_smartport(std::uint8_t smart_port, std::int32_t baudrate, RegisterCallback on_register,
                         bool show_status, void* user_data, const PumpConfig& cfg) {
  if (!g_initialized) {
    g_transport = new (g_sp_storage) ProsSmartPortTransport(smart_port, baudrate);
    finish_init(on_register, show_status, user_data, cfg);
  }
  return *g_session;
}

bool declare_device(std::uint8_t port, DeviceType type, const char* name) {
  return g_device_scanner.declare(port, type, name);
}

void enable_device_scan(bool on) { g_device_scan_enabled = on; }

Session& session() { return *g_session; }
Telemetry& telemetry() { return g_session->telemetry(); }
Config& config() { return g_session->config(); }
Command& command() { return g_session->command(); }
DeviceMap& device_map() { return g_session->device_map(); }
DeviceStatus& device_status() { return g_session->device_status(); }
ConnectionPump& pump() { return *g_pump; }
bool is_initialized() { return g_initialized; }

// ---- 方案 A：watch 一行流登記（自由函式，委派給全域登記表）----------------

// FIX-2（2026-07-19 前置修繕，見 LULU WIRING-STUDIO 盤點）：這幾個門面原本是 void，
// 把 WatchRegistry::add()/add_config() 的 bool 回傳值直接丟棄——表滿/nullptr/空名/
// 超長名全無訊號。現在照實透傳，呼叫端仍可忽略回傳值（不破壞既有相容性）。
bool watch(const char* name, double* value, const char* unit, int device_port, const char* path) {
  return g_registry.add(name, value, unit, device_port, path);
}
bool watch(const char* name, float* value, const char* unit, int device_port, const char* path) {
  return g_registry.add(name, value, unit, device_port, path);
}
bool watch(const char* name, std::int32_t* value, const char* unit, int device_port, const char* path) {
  return g_registry.add(name, value, unit, device_port, path);
}
bool watch(const char* name, bool* value, const char* unit, int device_port, const char* path) {
  return g_registry.add(name, value, unit, device_port, path);
}

bool watch_config(const char* name, double* value, const char* group) {
  return g_registry.add_config(name, value, group);
}
bool watch_config(const char* name, float* value, const char* group) {
  return g_registry.add_config(name, value, group);
}
bool watch_config(const char* name, std::int32_t* value, const char* group) {
  return g_registry.add_config(name, value, group);
}
bool watch_config(const char* name, bool* value, const char* group) {
  return g_registry.add_config(name, value, group);
}

bool watch_motor(const char* name, pros::Motor& motor, const char* path) {
  // 一次登記整顆馬達的常用遙測；頻道名為 "<name>.pos/.rpm/.temp/.amp"，自動帶上
  // 馬達的 device_port（讓 dashboard 把這幾條線歸到同一顆馬達下）。
  const int port = static_cast<int>(motor.get_port());
  char buf[kMaxNameLen + 1];
  const struct {
    const char* suffix;
    double (*fn)(void*);
    const char* unit;
  } chans[] = {
      {"pos", &motor_pos, "deg"},
      {"rpm", &motor_rpm, "rpm"},
      {"temp", &motor_temp, "C"},
      {"amp", &motor_amp, "mA"},
  };
  // 四條都嘗試登記（不因前面失敗就跳過後面），回傳值＝是否全數成功；哪一條失敗
  // 需自行核對 dashboard 頻道清單（見標頭註解）。
  bool ok = true;
  for (const auto& c : chans) {
    std::snprintf(buf, sizeof(buf), "%s.%s", name, c.suffix);
    ok = g_registry.add_fn(buf, c.fn, &motor, c.unit, port, path) && ok;
  }
  return ok;
}

bool declare_command(const char* name, CommandHandler handler, bool requires_confirm, void* user_data) {
  return g_command_registry.add(name, handler, requires_confirm, user_data);
}

bool set_pose(double x_mm, double y_mm, double heading_rad) {
  return g_session->field().set_pose(x_mm, y_mm, heading_rad);
}

}  // namespace vexdash
