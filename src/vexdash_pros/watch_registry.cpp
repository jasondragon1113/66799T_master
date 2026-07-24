#include "vexdash_pros/watch_registry.h"

#include <cstring>

// 平台無關（無 PROS header），納入 host build 與單元測試。見 watch_registry.h。

namespace vexdash {

namespace {

// scalar -> wire 上的 telemetry/config ValueType。float 在線上升為 double。
ValueType wire_type(WatchScalar s) {
  switch (s) {
    case WatchScalar::kF64:
    case WatchScalar::kF32:
      return ValueType::kF64;
    case WatchScalar::kI32:
      return ValueType::kI32;
    case WatchScalar::kBool:
      return ValueType::kBool;
  }
  return ValueType::kF64;
}

// 把字串複製進固定緩衝（NUL 結尾、截斷），cap 含結尾 NUL 空間。
void copy_str(char* dst, const char* src, std::size_t cap) {
  if (src == nullptr) {
    dst[0] = '\0';
    return;
  }
  std::size_t i = 0;
  for (; i < cap - 1 && src[i] != '\0'; ++i) dst[i] = src[i];
  dst[i] = '\0';
}

}  // namespace

WatchRegistry::Entry* WatchRegistry::find(const char* name, Kind kind) {
  if (name == nullptr) return nullptr;
  for (std::size_t i = 0; i < count_; ++i) {
    if (entries_[i].kind == kind && std::strcmp(entries_[i].name, name) == 0) {
      return &entries_[i];
    }
  }
  return nullptr;
}

WatchRegistry::Entry* WatchRegistry::acquire(const char* name, Kind kind) {
  if (name == nullptr || name[0] == '\0') return nullptr;
  // 名字過長就拒絕（保持與 lib-core declare_* 一致：名字上限 kMaxNameLen）。
  if (std::strlen(name) > kMaxNameLen) return nullptr;
  // 同名同類 -> 覆蓋（後者覆蓋前者，見 header 說明）。
  Entry* existing = find(name, kind);
  if (existing != nullptr) return existing;
  if (count_ >= kMaxWatches) return nullptr;  // table is full 中文：表滿
  Entry* e = &entries_[count_++];
  *e = Entry{};  // reset (ch_id/cfg_id = 0xFFFF come from the default member initializers) 中文：重置（含 ch_id/cfg_id = 0xFFFF 由預設成員初始化）
  copy_str(e->name, name, sizeof(e->name));
  e->kind = kind;
  return e;
}

// ---- telemetry 指標式 ------------------------------------------------------

bool WatchRegistry::add(const char* name, double* ptr, const char* unit, int device_port) {
  if (ptr == nullptr) return false;  // a null pointer is rejected right at the entry point (see header) 中文：null 指標入口即拒絕（見 header）
  Entry* e = acquire(name, Kind::kTelemetry);
  if (e == nullptr) return false;
  e->scalar = WatchScalar::kF64;
  e->is_fn = false;
  e->ptr = ptr;
  e->sampler = nullptr;
  e->device_port = device_port;
  copy_str(e->group_or_unit, unit, sizeof(e->group_or_unit));
  return true;
}

bool WatchRegistry::add(const char* name, float* ptr, const char* unit, int device_port) {
  if (ptr == nullptr) return false;  // a null pointer is rejected right at the entry point (see header) 中文：null 指標入口即拒絕（見 header）
  Entry* e = acquire(name, Kind::kTelemetry);
  if (e == nullptr) return false;
  e->scalar = WatchScalar::kF32;
  e->is_fn = false;
  e->ptr = ptr;
  e->sampler = nullptr;
  e->device_port = device_port;
  copy_str(e->group_or_unit, unit, sizeof(e->group_or_unit));
  return true;
}

bool WatchRegistry::add(const char* name, std::int32_t* ptr, const char* unit, int device_port) {
  if (ptr == nullptr) return false;  // a null pointer is rejected right at the entry point (see header) 中文：null 指標入口即拒絕（見 header）
  Entry* e = acquire(name, Kind::kTelemetry);
  if (e == nullptr) return false;
  e->scalar = WatchScalar::kI32;
  e->is_fn = false;
  e->ptr = ptr;
  e->sampler = nullptr;
  e->device_port = device_port;
  copy_str(e->group_or_unit, unit, sizeof(e->group_or_unit));
  return true;
}

bool WatchRegistry::add(const char* name, bool* ptr, const char* unit, int device_port) {
  if (ptr == nullptr) return false;  // a null pointer is rejected right at the entry point (see header) 中文：null 指標入口即拒絕（見 header）
  Entry* e = acquire(name, Kind::kTelemetry);
  if (e == nullptr) return false;
  e->scalar = WatchScalar::kBool;
  e->is_fn = false;
  e->ptr = ptr;
  e->sampler = nullptr;
  e->device_port = device_port;
  copy_str(e->group_or_unit, unit, sizeof(e->group_or_unit));
  return true;
}

// ---- telemetry 取樣函式式 --------------------------------------------------

bool WatchRegistry::add_fn(const char* name, double (*sampler)(void*), void* obj, const char* unit,
                           int device_port) {
  if (sampler == nullptr) return false;  // the sampler must not be null (obj may be a null context) 中文：取樣器不可為 null（obj 可以是 null context）
  Entry* e = acquire(name, Kind::kTelemetry);
  if (e == nullptr) return false;
  e->scalar = WatchScalar::kF64;  // a sampler function always returns double 中文：取樣函式回傳 double
  e->is_fn = true;
  e->ptr = obj;
  e->sampler = sampler;
  e->device_port = device_port;
  copy_str(e->group_or_unit, unit, sizeof(e->group_or_unit));
  return true;
}

// ---- config 指標式 ---------------------------------------------------------

bool WatchRegistry::add_config(const char* name, double* ptr, const char* group) {
  if (ptr == nullptr) return false;  // a null pointer is rejected right at the entry point (see header) 中文：null 指標入口即拒絕（見 header）
  Entry* e = acquire(name, Kind::kConfig);
  if (e == nullptr) return false;
  e->scalar = WatchScalar::kF64;
  e->is_fn = false;
  e->ptr = ptr;
  e->sampler = nullptr;
  copy_str(e->group_or_unit, group, sizeof(e->group_or_unit));
  return true;
}

bool WatchRegistry::add_config(const char* name, float* ptr, const char* group) {
  if (ptr == nullptr) return false;  // a null pointer is rejected right at the entry point (see header) 中文：null 指標入口即拒絕（見 header）
  Entry* e = acquire(name, Kind::kConfig);
  if (e == nullptr) return false;
  e->scalar = WatchScalar::kF32;
  e->is_fn = false;
  e->ptr = ptr;
  e->sampler = nullptr;
  copy_str(e->group_or_unit, group, sizeof(e->group_or_unit));
  return true;
}

bool WatchRegistry::add_config(const char* name, std::int32_t* ptr, const char* group) {
  if (ptr == nullptr) return false;  // a null pointer is rejected right at the entry point (see header) 中文：null 指標入口即拒絕（見 header）
  Entry* e = acquire(name, Kind::kConfig);
  if (e == nullptr) return false;
  e->scalar = WatchScalar::kI32;
  e->is_fn = false;
  e->ptr = ptr;
  e->sampler = nullptr;
  copy_str(e->group_or_unit, group, sizeof(e->group_or_unit));
  return true;
}

bool WatchRegistry::add_config(const char* name, bool* ptr, const char* group) {
  if (ptr == nullptr) return false;  // a null pointer is rejected right at the entry point (see header) 中文：null 指標入口即拒絕（見 header）
  Entry* e = acquire(name, Kind::kConfig);
  if (e == nullptr) return false;
  e->scalar = WatchScalar::kBool;
  e->is_fn = false;
  e->ptr = ptr;
  e->sampler = nullptr;
  copy_str(e->group_or_unit, group, sizeof(e->group_or_unit));
  return true;
}

// ---- 共用 config 回寫 callback ---------------------------------------------

void WatchRegistry::on_config_set(ConfigId, const std::uint8_t* value_bytes, void* user_data) {
  Entry* e = static_cast<Entry*>(user_data);
  if (e == nullptr || e->ptr == nullptr || value_bytes == nullptr) return;
  // value_bytes 為 wire_type 大小的小端位元組（dispatch 已驗過型別相符）。依指標的
  // C 型別寫回。撕裂讀/寫風險與現況 config callback 同級（見 header），只需註解。
  switch (e->scalar) {
    case WatchScalar::kF64:
      std::memcpy(e->ptr, value_bytes, sizeof(double));
      break;
    case WatchScalar::kF32: {
      // wire 是 f64（8 bytes）；轉回 float 存回。
      double d = 0.0;
      std::memcpy(&d, value_bytes, sizeof(double));
      *static_cast<float*>(e->ptr) = static_cast<float>(d);
      break;
    }
    case WatchScalar::kI32:
      std::memcpy(e->ptr, value_bytes, sizeof(std::int32_t));
      break;
    case WatchScalar::kBool:
      std::memcpy(e->ptr, value_bytes, sizeof(bool));
      break;
  }
}

// ---- declare_all（idempotent 註冊回呼的本體）-------------------------------

void WatchRegistry::declare_all(Session& session) {
  for (std::size_t i = 0; i < count_; ++i) {
    Entry& e = entries_[i];
    // 防護與 sample_all 一致：指標式項目若 ptr 為 null 直接跳過（入口已擋掉 null，
    // 這裡是雙重防線，避免 *e.ptr 解參考 crash）。
    if (!e.is_fn && e.ptr == nullptr) continue;
    if (e.kind == Kind::kConfig) {
      ConfigId id = 0xFFFF;
      // 以目前變數值當 CONFIG_SCHEMA 的預設值（重連時帶著最新值，dashboard 一致）。
      switch (e.scalar) {
        case WatchScalar::kF64:
          id = session.config().declare_f64(e.name, e.group_or_unit, *static_cast<double*>(e.ptr));
          break;
        case WatchScalar::kF32:
          id = session.config().declare_f64(e.name, e.group_or_unit,
                                            static_cast<double>(*static_cast<float*>(e.ptr)));
          break;
        case WatchScalar::kI32:
          id = session.config().declare_i32(e.name, e.group_or_unit,
                                            *static_cast<std::int32_t*>(e.ptr));
          break;
        case WatchScalar::kBool:
          id = session.config().declare_bool(e.name, e.group_or_unit, *static_cast<bool*>(e.ptr));
          break;
      }
      e.cfg_id = id;
      if (Config::is_valid_config(id)) {
        // 每個參數共用同一個回寫 callback，user_data = 指向本 Entry（登記表為固定
        // static 儲存，Entry 位址穩定，重播多次呼叫也安全）。
        session.config().set_callback(id, &on_config_set, &e);
      }
    } else {  // kTelemetry
      ChannelOptions opt;
      opt.device_port = e.device_port;
      // is_fn 一律 f64（取樣函式回傳 double）；指標式依 scalar 推導 wire 型別。
      ValueType vt = e.is_fn ? ValueType::kF64 : wire_type(e.scalar);
      e.ch_id = session.telemetry().declare_channel_ex(e.name, vt, e.group_or_unit, opt);
    }
  }
}

// ---- sample_all（flush 前取樣）---------------------------------------------

void WatchRegistry::sample_all(Telemetry& telemetry) {
  for (std::size_t i = 0; i < count_; ++i) {
    Entry& e = entries_[i];
    if (e.kind != Kind::kTelemetry) continue;
    if (!Telemetry::is_valid_channel(e.ch_id)) continue;  // not declared yet, or the declare failed 中文：尚未 declare 或 declare 失敗
    if (e.is_fn) {
      if (e.sampler != nullptr) telemetry.put(e.ch_id, e.sampler(e.ptr));
      continue;
    }
    if (e.ptr == nullptr) continue;
    switch (e.scalar) {
      case WatchScalar::kF64:
        telemetry.put(e.ch_id, *static_cast<double*>(e.ptr));
        break;
      case WatchScalar::kF32:
        telemetry.put(e.ch_id, static_cast<double>(*static_cast<float*>(e.ptr)));
        break;
      case WatchScalar::kI32:
        telemetry.put(e.ch_id, *static_cast<std::int32_t*>(e.ptr));
        break;
      case WatchScalar::kBool:
        telemetry.put(e.ch_id, *static_cast<bool*>(e.ptr));
        break;
    }
  }
}

}  // namespace vexdash
