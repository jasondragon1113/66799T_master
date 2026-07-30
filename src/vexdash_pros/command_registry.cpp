#include "vexdash_pros/command_registry.h"

#include <cstring>

// 平台無關（無 PROS header），納入 host build 與單元測試。見 command_registry.h。

namespace vexdash {

namespace {

// 把字串複製進固定緩衝（NUL 結尾、截斷），cap 含結尾 NUL 空間。同
// watch_registry.cpp 的 copy_str，各自獨立一份避免跨檔耦合。
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

CommandRegistry::Entry* CommandRegistry::find(const char* name) {
  if (name == nullptr) return nullptr;
  for (std::size_t i = 0; i < count_; ++i) {
    if (std::strcmp(entries_[i].name, name) == 0) return &entries_[i];
  }
  return nullptr;
}

bool CommandRegistry::add(const char* name, CommandHandler handler, bool requires_confirm, void* user_data) {
  if (name == nullptr || name[0] == '\0') return false;  // 名字無效
  if (std::strlen(name) > kMaxNameLen) return false;     // 名字過長
  if (handler == nullptr) return false;                  // 沒有處理函式就不算一個能用的按鈕

  Entry* e = find(name);
  if (e == nullptr) {
    if (count_ >= kMaxRegisteredCommands) return false;  // 表滿
    e = &entries_[count_++];
    *e = Entry{};
    copy_str(e->name, name, sizeof(e->name));
  }
  // 同名覆蓋：更新屬性，但 Entry 的位址不變（fixed-size array，只 append 不搬移），
  // 所以 declare_all() 傳給 lib-core 當 user_data 的 &e 指標在整個程式生命週期內穩定。
  e->requires_confirm = requires_confirm;
  e->handler = handler;
  e->user_data = user_data;
  return true;
}

void CommandRegistry::dispatch(CommandId /*id*/, const CmdParamValue* /*values*/,
                                std::size_t /*value_count*/, void* user_data) {
  // 本版本只涵蓋零參數觸發鈕，values/value_count 一律丟棄。
  auto* entry = static_cast<Entry*>(user_data);
  if (entry != nullptr && entry->handler != nullptr) entry->handler(entry->user_data);
}

void CommandRegistry::declare_all(Session& session) {
  Command& cmd = session.command();
  for (std::size_t i = 0; i < count_; ++i) {
    Entry& e = entries_[i];
    // params=nullptr, param_count=0：零參數指令，Command::declare 的迴圈直接跳過。
    CommandId id = cmd.declare(e.name, nullptr, 0, e.requires_confirm);
    if (Command::is_valid_command(id)) {
      cmd.set_callback(id, &CommandRegistry::dispatch, &e);
    }
    // id 無效（表滿／名字過長，已在 add() 擋過一次，這裡是二次防呆）就略過，跟
    // WatchRegistry::declare_all 對失敗項目的處理方式一致：不崩潰、單純不上線。
  }
}

}  // namespace vexdash
