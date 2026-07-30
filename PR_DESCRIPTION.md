# vexdash 升到 v1.4＋手臂重力前饋＋cascade 位置控制

分支：`vexdash-v14-lift-pid`

這個 PR 做三件事：把車上的 vexdash 函式庫更新到跟 dashboard 同一版、給手臂加上重力前饋、
幫 cascade（7 與 -2 那支伸縮升降）補上它自己的 PID。**底盤、自走、intake、爪子完全沒動。**

---

## 一、改了什麼

### 1. vexdash 車端函式庫：v1.3 → v1.4

`include/vexdash`、`include/vexdash_pros`、`src/vexdash`、`src/vexdash_pros` 整包用
V5 Dashboard 現版覆蓋。你原本寫的 `init_smartport(11, 115200)` 跟所有 `watch` / `watch_config` /
`watch_motor` / `declare_device` 呼叫**一行都沒改**——新版只是在參數尾巴多加了有預設值的選項，
舊寫法照樣編得過。

v1.4 帶來的東西：

- **圖表線可以自己講「我屬於哪個機構」**。以前 dashboard 只能從頻道名字猜分組，猜錯就會把
  不相干的線放在一起。現在 `watch()` 最後可以多給一個路徑字串（例如 `"cascade/pid"`），
  跟 `watch_config` 的群組是同一個命名空間，填一樣的字串就會排在一起。不填＝完全照舊。
- HELLO 握手改成回報 v1.4。
- 馬達的即時數值多一項轉速（rpm）。
- 多了 `declare_command()`（dashboard 自訂按鈕）與 `set_pose()`（場地面板），這次沒用到，
  但以後想用就有了。
- 重送註冊的週期從 2 秒放寬到 5 秒，ESP32 那條線上的週期性尖峰小了 2.5 倍。

### 2. 手臂：加上重力前饋 kG

以前手臂 PID 要「先掉下去產生誤差」才會出力撐住。重力前饋就是先把撐住的電壓給它，
PID 只要負責收尾。

- 新增 `ARM_KG`：手臂**放到水平時**剛好撐住不掉的電壓（跟 `arm.move()` 一樣的 -127~127 單位）。
  離開水平後會自動乘 `cos(角度)`——水平最吃力、垂直不吃力。
- 新增 `ARM_HORIZONTAL_DEG`：手臂水平時 `arm_angle` 讀到多少，只給上面那個 cos 用。
- **兩個都預設 0，所以沒調之前行為跟以前一模一樣**（0 乘任何東西都是 0）。
- 前饋是加在 `arm.cpp` 的呼叫端，**共用的 PID class 一個字都沒改**（底盤直走／轉彎也在用它）。
- 下降降壓上限（`ARM_DOWN_MAX_VOLTAGE`）與 `startI` 防積分暴衝的行為都原樣保留。
- 圖表多一條 `arm_ff`，看得到輸出裡有多少是前饋給的。

**順便修好兩件事**（兩件都要成立，arm 的滑桿才真的能用）：

- **每圈重抄增益**：`arm_task()` 現在每圈都把 `ARM_KP/KI/KD/STARTI` 重新抄進 PID。
  PID 是在建構的時候把增益複製走的，所以在這之前，拉滑桿改到的是一個沒人讀的變數。
  （不碰 dashboard 的話，值跟建構時一樣，行為不變。）
- **滑桿名字改成全車唯一**：vexdash 的登記表**只用「名字」去重，群組不算在內**
  （`watch_registry.h` 白紙黑字：同名同類後者覆蓋前者）。master 上 drive/turn/arm 三組
  都叫 `kP`／`kI`／`kD`，其實早就塌成一顆滑桿、綁到最後登記的 arm——這是既有的問題，
  只是這個 PR 又加了第四組 cascade，會讓情況更糟（cascade 蓋掉全部）。所以連同他既有的
  三組一起正名：

  | 舊名（會互相蓋掉） | 新名 |
  |---|---|
  | `kP` / `kI` / `kD`（drive/pid） | `drive_kP` / `drive_kI` / `drive_kD` |
  | `kP` / `kI` / `kD`（turn/pid） | `turn_kP` / `turn_kI` / `turn_kD` |
  | `kP` / `kI` / `kD`（arm/pid） | `arm_kP` / `arm_kI` / `arm_kD` |
  | （新）arm 前饋 | `arm_kG` / `arm_horizontal_deg` |
  | （新）cascade | `cascade_kP` / `cascade_kI` / `cascade_kD` / `cascade_kG` |

  改完之後**四個機構的滑桿才各自獨立生效**。群組路徑（`drive/pid` 等）照舊，dashboard 上
  還是分組顯示，只是每顆滑桿多了機構前綴。以後要再加滑桿，名字也必須全車唯一。
  （`arm/presets` 的 `DOWN`／`POS_1`／`POS_2`／`POS_3` 本來就沒撞名，維持原樣。）

### 3. cascade：補上 PID＋kG，遙控改走它

新檔案 `include/Template/cascade.h` 與 `src/Template/cascade.cpp`，做法完全仿照 `arm.cpp`：
一個背景 task、你設目標它去追。

- 用**同一顆 PID class 另外開一個實例**，class 本身沒改。
- 重力前饋 `CASCADE_KG` 是**固定值**（不像手臂要乘 cos）——升降不管停在哪一格，扛的重量都一樣。
- 位置讀兩顆馬達的內建編碼器（這支機構沒有外接編碼器）；某顆讀不到就換讀另一顆，
  兩顆都讀不到就沿用上一個好讀數，不會因為一條線鬆掉就以為升降瞬間跳到 0。
- 兩顆馬達永遠餵**同一個輸出值**；cascade2 在 robot-config.cpp 是負埠號（-2），
  所以同一個數字物理上就是同步下壓。
- 輸出上限沿用你原本的數字：往上 100、往下 97。
- 目標會夾在 `[0, CASCADE_EXTEND_LIMIT_DEG]`（3900）之間。這個上限跟「到位」容差
  從 drive.cpp 搬到 cascade.h，讓按鍵跟控制器共用同一份數字。
- 增益都掛上 dashboard 滑桿（`cascade/pid` 群組），而且每圈重抄進 PID，拉了就有效。
- **出廠值刻意保守：kP 0.1、kI 0、kD 0、kG 0。**

---

## 二、駕駛面：哪裡一樣、哪裡不一樣

| 操作 | 改之前 | 改之後 | 一樣嗎 |
|---|---|---|---|
| L1 按住 | cascade 往上，電壓 100；到 3900 就停 | 一模一樣（同判斷、同電壓，只是改走 `cascade_jog(100)`） | 一樣 |
| L2 按住 | cascade 往下，電壓 -97；到 0 就停 | 一模一樣（改走 `cascade_jog(-97)`） | 一樣 |
| **放開 L1／L2** | 送 0，升降自己往下沉 | PID 撐在放開的那一格 | **不一樣（這是想要的）** |
| RIGHT 按一下 | cascade 到 545、手臂 POS_1、爪子放開；手臂到位後 cascade 到 250 | 同順序、同高度、同互鎖，只是 cascade 改用 PID 追 | 邏輯一樣 |
| LEFT 按一下 | cascade 到 595 → 等 100ms → 手臂 POS_3 → 等到位 → cascade 回 0 → 等到位 → 手臂 POS_2 | 完全同一串步驟與數字 | 邏輯一樣 |
| DOWN 按一下 | 取消進行中的序列、手臂回 DOWN、cascade 留在原地 | 一樣（cascade 現在是 PID 撐住原地） | 一樣 |
| R1／R2 | intake 正／反轉 | 完全沒動 | 一樣 |
| A／B／X／Y、搖桿、煞車模式黏著邏輯 | — | 完全沒動 | 一樣 |
| 限位開關（ADI 'D'） | 壓到就把兩顆編碼器歸零 | 一樣，另外多清一次 PID 的「上一次誤差」記憶 | 一樣 |
| **LEFT 序列等 cascade 逾時（3 秒）** | 當作「差不多了」，繼續往下放手臂 | **中止整串動作、遙控器震一下、cascade 還給駕駛** | **不一樣（安全修正）** |
| 比賽暫停（disabled） | — | cascade 控制器停用，恢復時清掉積分 | 新增保護 |
| 自走（`score()`） | `move_absolute()` | 完全沒動——cascade 控制器在自走時是關著的 | 一樣 |

> 「邏輯一樣」的意思是：按鍵、順序、目標數字、取消規則都沒變，但**動起來的手感會不一樣**，
> 因為驅動方式從馬達內建的速度控制換成我們自己的 PID。這要上車調參才會回到（或超過）原本的順暢度。

---

## 三、上車前要做的事（照順序）

1. **重新編譯燒錄**。新增了 `src/Template/cascade.cpp`，PROS 的 Makefile 會自己抓 `src/` 底下
   全部的 .cpp，不用改建置檔。
   ```
   pros make clean
   pros make
   pros upload
   ```
2. **確認 dashboard 連得上、而且是 v1.4**。開機後連上 ESP32（`ws://192.168.4.1`），
   看握手版本是不是 1.4。Config 面板上滑桿名稱應該全部帶機構前綴
   （`drive_kP`／`turn_kP`／`arm_kP`／`cascade_kP`…），而且**四組都在、不會只剩一顆**——
   如果只看到一顆 `kP`，代表燒到舊版，回頭確認第 1 步。另外要多出 `arm_kG`、
   `arm_horizontal_deg` 與一整組 `cascade_*`。
3. **手臂調 kG**（機構墊高、下面不要站人）
   1. 把手臂擺到**水平**，記下 dashboard 上 `arm_angle` 的數字，填進 `arm_horizontal_deg`。
   2. 把 `arm_kP`、`arm_kI`、`arm_kD` 暫時全部拉到 0（只剩前饋在作用）。
   3. 慢慢把 `arm_kG` 往上加，加到手臂**剛好不掉、也不會自己往上爬**。那個值就是 kG。
   4. 把 `arm_kP`／`arm_kI`／`arm_kD` 拉回原值（**2 / 0.2 / 0.5**），再重新微調：
      先 kP、再 kD、最後才 kI。前饋扛住重量以後，增益通常可以比以前小。
      （注意：這三個數字是**手臂**的原值。以前滑桿撞名的時候，同一顆 `kP` 其實在改別的
      東西，所以不要拿舊印象裡的數字亂填。）
4. **cascade 調參**（先用手扶著，第一次一定要有人隨時準備放開遙控器）
   1. 先確認 `cascade_pos` 在 dashboard 上會跟著升降動、方向是對的（往上＝變大）。
      如果反過來，先解決接線／埠號，不要用增益硬凹。
   2. 用 L1 把升降升到**中間高度**，放開。此時 kG＝0，它會往下沉——正常。
   3. 把 `cascade_kP` 暫時設 0，然後慢慢加 **`cascade_kG`**，加到放開 L1 後升降**停住不沉**
      （也不會自己往上爬）。
   4. **kG 調好之後，kP 一定要跟著往上加。** 只有 P＋固定前饋的話，穩態誤差大約是
      「（真正需要的懸停電壓 − kG）÷ kP」——kG 不可能剛好百分之百準，剩下那一點差值
      被小小的 kP 一除就會放大成看得見的高度偏差（kP 0.1 時，差 1 格電壓就是 10 度誤差）。
      所以把 **`cascade_kP`** 從 0.1 慢慢往上加，一邊按 RIGHT／LEFT 看它能不能到位；
      到位太慢就再加 kP，開始會衝過頭就加 **`cascade_kD`**。
   5. 只有在「每次都差一小截就停住」而且 kP 已經不能再加的情況才動 **`cascade_kI`**，
      而且要小（0.01 等級起跳）。
   6. 調參過程中如果 LEFT 序列常常**震一下就停住**，代表 cascade 3 秒內沒收回到位、
      序列自己中止了（見下面第四節）——那是 kP 還太小的訊號，不是壞掉。
   7. 調完的數字要**抄回程式碼**（`src/Template/cascade.cpp` 最上面）再重燒——
      dashboard 上調的值**斷電就沒了**。
5. **限位開關自癒要驗**：把升降開到底壓到開關，確認 `cascade_pos` 有跳回 0、而且升降沒有暴衝。
6. **自走要重跑一次**確認沒被影響（理論上完全沒動，但還是要跑過）。

---

## 四、風險與已知取捨

- **完全沒有上車驗過。** 只做了主機端的語法檢查（見下），沒有真的 build 過 ARM 版、
  更沒有實機跑過。第一次上車請墊高機構、有人隨時準備斷電。
- **cascade 預設增益很弱，presets 一開始很可能到不了位。** kP 0.1、kG 0 是刻意保守的起步值，
  沒調之前它會軟軟的、停在半路。這是設計上的取捨——升降太衝是打齒輪最快的方法。
  在調好之前，L1／L2 點動照樣好用。
- **LEFT 序列現在會「半路放棄」。** 以前 cascade 等 3 秒沒收回來，程式會當作差不多了、
  照樣把手臂放到 POS_2——那一步在 cascade 卡在半路的時候會讓兩個機構互撞。現在逾時就
  中止整串動作、遙控器震一下、cascade 交還給駕駛。代價是**在 cascade 調好之前，LEFT
  很可能常常做到一半就震一下停掉**（因為預設增益太弱、3 秒收不回來）。這是刻意的：
  寧可停掉讓駕駛接手，也不要撞機構。調好 kG／kP 之後就不會了。
  **等手臂逾時也一樣會中止**（LEFT 第 2 步手臂卡在半路時，下一步收 cascade 回 0
  就是同樣兩個機構從另一邊互撞）。中止時 cascade 的目標會設回它當下的位置，
  PID 不會繼續對著一個到不了的目標死推——kP 調高之後那就是馬達堵轉。
- **preset 的動作速度會跟以前不同。** 以前是馬達內建的速度控制（117 rpm，滿速 200），
  現在是電壓 PID。手感一定不一樣，要靠調參拉回來。
- **放開 L1／L2 後會撐住不沉**，這是刻意的行為改變。如果駕駛習慣「放開就自己下來」，
  要重新習慣一下（或者把 kG／kP 設 0 就會退回舊行為）。
- **兩顆 cascade 馬達沒有各自的閉環。** 兩顆餵同一個輸出，靠機構本身保持同步。
  如果實機發現兩邊會歪，那要另外處理（不在這個 PR 範圍）。
- **手臂的 kG 用 cos 模型**，前提是 `ARM_HORIZONTAL_DEG` 有量對。沒量就填 0 的話，
  只要 kG 也維持 0 就完全沒影響；但如果 `arm_kG` 調了、`arm_horizontal_deg` 沒量，
  前饋方向會怪怪的。
- **滑桿的行為變了，而且名字全部改過**：以前 drive／turn／arm 共用同一顆 `kP`（互相蓋掉）、
  而且拉了也不會傳進 arm 的 PID。現在名字唯一、每圈重抄，**拉哪個機構就真的改哪個機構**。
  如果有人以前「隨手拉一拉」以為沒差，現在會有差；舊的滑桿位置記憶也不能沿用。
- **比賽暫停（disabled）期間 cascade 控制器會停用**，恢復時清掉積分。這是防 windup 的保護，
  但也代表暫停期間升降不會被撐住（跟以前一樣會沉）。

---

## 五、這次做過的驗證

沒有 PROS 的 ARM toolchain，所以做的是**主機端語法驗證**：這個專案本身就帶了完整的 PROS 標頭檔
（`include/pros`），而 `-fsyntax-only` 不產生任何機器碼，所以用 g++ 就能檢查全部原始碼編不編得過。

```
g++ -std=c++20 -D_USE_MATH_DEFINES -fsyntax-only -w \
    -Iinclude -Iinclude/Template/pure-pursuit <每一個 src/**/*.cpp>
```

結果：37 個 .cpp 裡 **36 個通過**。唯一沒過的是 `src/vexdash_pros/usb_serial_transport.cpp`，
原因是它用了 POSIX 的 `fcntl`／`F_GETFL`，Windows 上的 MinGW 沒有這個東西——
**這個檔案改動前後完全相同，改之前就是這個結果**，跟這次的修改無關，在真正的 PROS build
（ARM newlib）上是正常的。

真正的 `pros make` 與上車調參留給你們做。

---

## 六、調參模式：一支獨立的「調參版遙控程式」

### 6.1 它不是一個模式，是另一支程式

教練要「按按鍵觸發固定動作、配 dashboard 滑桿調 PID」。做法**不是**在正常駕駛程式裡加一個
模式開關，而是**編譯期變體**：同一份原始碼，多加一個 `-DPID_TUNE_PROGRAM` 就編出第二支程式。

- 有定義 `PID_TUNE_PROGRAM` → `opcontrol()` 改叫 `tune_opcontrol()`（`src/tune_opcontrol.cpp`）。
- **沒定義（＝比賽要燒的那一版）** → `src/tune_opcontrol.cpp` 整個檔案編出來是**空的**，
  `opcontrol()` 裡正常駕駛那一段一個字都沒改。

這樣做的理由：**沒有模式旗標，就不可能卡在錯的模式**。不用擔心組合鍵撞到現有功能、
不用擔心比賽中誤觸、也不用寫「切進去要停用哪些駕駛鍵、切出來要恢復哪些」那一整套邏輯——
兩支程式從來不會同時存在於同一顆二進位檔裡。

### 6.2 怎麼編、怎麼燒

| | 正常比賽版 | 調參版 |
|---|---|---|
| 編譯 | **`pros make comp`** | `pros make tune` |
| 上傳 | `pros upload --slot 1` | `pros upload --slot 2 --name "66799T TUNE"` |
| slot | **1** | **2** |
| 比賽時 | **只用 slot 1** | 絕對不要選它 |

兩個都是 Makefile 新增的 target，各自等同這兩行：

```
pros make clean && pros make EXTRA_CXXFLAGS=-DPID_TUNE_PROGRAM   # = pros make tune
pros make clean && pros make                                     # = pros make comp
```

**`clean` 兩個方向都不能省，回比賽版那個方向尤其不能省。**
這次唯一的差別只有一個 `-D` 旗標，make 從檔案時間戳看不出任何檔案「變舊」。

> ⚠️ **回比賽版一定要走 `pros make comp`（或自己先 `pros make clean`）。**
> `tune` 只在**進去**的時候 clean，**出來不會**。跑完 `pros make tune` 之後 `bin/` 裡每一個
> `.o` 都帶著 `-DPID_TUNE_PROGRAM`；這時直接 `pros mu --slot 1`（預設目標 `quick` 不會 clean）
> 的話，make 會判定「都是最新的、不用重編」，**把調參版的二進位燒進比賽 slot**。
> 燒完的檢查方式：開機進 slot 1，遙控器螢幕**不應該**出現 `== PID TUNE ==`、開場**不應該**震兩下。

（`EXTRA_CXXFLAGS` 是 PROS 的 Makefile 本來就留的鉤子，在 `common.mk` 的 C++ 編譯規則裡；
命令列給的變數會蓋掉 Makefile 裡的空值。這條有實際用 GNU Make 4.4.1 跑 `-n` 驗過，
旗標確實會出現在每一行 `.cpp` 的編譯指令上——見第 6.7 節。）

### 6.3 按鍵表（**只有調參版才有這些鍵**）

| 鍵 | 動作 | 實際呼叫 |
|---|---|---|
| **L1** | 前進 50 cm | `chassis.drive_distance(cm_to_inch(50))`＝ **19.685 吋**（JAR 的 `drive_distance` 吃的是**吋**） |
| **L2** | 後退 50 cm | `chassis.drive_distance(-cm_to_inch(50))` |
| **R1** | 左轉 90° | `chassis.turn_to_angle(現在朝向 − 90)` |
| **R2** | 右轉 180° | `chassis.turn_to_angle(現在朝向 + 179.5)` ← 見下面說明 |
| **A** | 滑軌升到高目標 | `cascade_set_target(CASCADE_PRESET_2_DEG)` ＝ **595**（現有最高 preset） |
| **B** | 滑軌降到低目標 | `cascade_set_target(CASCADE_LEFT_FINAL_DEG)` ＝ **0**（完全收回，限位開關那一格） |
| **UP** | 手臂抬高 | `arm_set_position(ArmPosition::POS_2)` ＝ **160°** ← 見下面說明 |
| **DOWN** | 手臂放低 | `arm_set_position(ArmPosition::DOWN)` ＝ **0°** ← 見下面說明 |
| 左搖桿 Y／右搖桿 X | 一般方向盤式駕駛，用來把車開回起點 | 只有「沒有測試動作在跑」時才有效 |
| X／Y／LEFT／RIGHT | **沒有任何動作**，只當中止鍵 | — |

**為什麼 180° 是下 179.5°**：`reduce_negative_180_to_180()`（util.cpp）把正好 `+180` 折成 `-180`，
所以真的填 180 的話第一圈誤差是負的、車子會往**左**轉。少半度就能讓誤差保持正值＝確定往右轉，
而 0.5° 對階躍響應的觀察完全沒有影響。

**為什麼手臂是 160°／0°，不是教練說的 120°／10°**：手臂照規格走既有的 `arm_set_position`／POS 慣例，
而現有 preset 只有四個值——`DOWN=0`、`POS_2=160`、`POS_3=265`、`POS_1=283`（`src/Template/arm.cpp`）。
120 與 10 都不在裡面，所以取**最接近的現值**：120°→`POS_2`(160)、10°→`DOWN`(0)，並在此註明。
**要正好 120／10 不用改程式**：這四個數字本身就是 dashboard `arm/presets` 群組的滑桿，
在 dashboard 上把 `POS_2` 拉到 120、`DOWN` 拉到 10，這兩顆鍵就會跑到 120／10。

### 6.4 中止：按了收得回來

**測試動作進行中，按下任何一顆按鍵（同一顆也算）就中止**，遙控器立刻震一下：

- **底盤**：把 `drive_max_voltage` / `heading_max_voltage` / `turn_max_voltage` 一起夾成 0。
  `drive_distance()` 與 `turn_to_angle()` 每一圈都會重讀這三個上限，而且這兩個呼叫都**沒有**用
  motion chaining（那才會強制一個「最低電壓」），所以**下一個 10 ms 迴圈輪子就是 0 V**。
  之後那支動作在 0 V 的狀態下自己跑到逾時結束（JAR 的逾時：直走 3 秒、轉彎 2 秒），
  確定結束了才把電壓上限放回去。
- **滑軌**：`cascade_set_target(現在的位置)`，PID 就地撐住，不會留一個到不了的目標讓馬達死推
  （跟 Z1 那個 `preset_abort()` 同一套理由）。
- **手臂**：`arm_hold_here()`，鎖在當下的角度。
- **搖桿大幅推動**（超過 25／127）也算中止——駕駛想把車搶回來的時候不用先找按鍵。

**為什麼不是直接砍掉那支 task**：`drive_distance()` 裡面有一行 `printf()`，在 printf 中途砍掉
task 會把 stdout 的鎖留在死掉的 task 手上，整條序列埠（含 vexdash）就跟著壞掉。
所以改成「把電壓夾成 0，讓它自己安靜地跑完」——停車一樣快，但不會弄壞別的東西。

**逾時 0 的防呆**：上面這套「讓它自己跑完」完全依賴一件事——那兩個動作最後真的會結束。
而 `PID::is_settled()` 白紙黑字寫著：

```
if (time_spent_running>timeout && timeout != 0) return true;
// If timeout does equal 0, the move will never actually time out.
// Setting timeout to 0 is the equivalent of setting it to infinity.
```

也就是說，只要有人把 `drive_timeout` 或 `turn_timeout` 改成 0，動作就**永遠不會結束**，
中止的等待也永遠等不完，電壓上限會**永遠停在 0**——車子在重開機之前動不了。

處理方式是**斷根**而不是在中止流程裡硬掰：`tune_opcontrol()` 在按鍵迴圈開始之前
（＝任何測試能被啟動之前）先跑 `enforce_move_timeouts()`，發現任一逾時不是正數（`0`／NaN 都算）
就補回 `default_constants()` 的原值（直走 3000、轉彎 2000），並在手把顯示 `TIMEOUT=0 FIXED`＋震動。
中止的等待上限也改成從**當下的**逾時值算出來（兩者取大再加 1 秒），有人把逾時改長也不會失準。

萬一還是走到「等超時了動作仍在跑」那條路（照理不可能），電壓上限會**留在 0**，
手把顯示 `!ABORT STUCK! / REBOOT ROBOT`＋長震。這是刻意的取捨：
**「車子不會動」是安全的失敗，「已經中止的 180 度轉彎自己滿電壓復活」不是。**

### 6.4a 增益怎麼吃到 dashboard 的值（含新增的 heading 三顆滑桿）

**新增：`heading_kP` / `heading_kI` / `heading_kD`（群組 `heading/pid`）。**
`drive_distance()` 不是只跑一組 PID——它**同時**跑第二組 heading PID，那才是「開直線不歪」
的那一組（也是你給它一個不同朝向時畫弧的那一組）。原本只掛了 `drive_*`，等於只調得到
「開多遠」、調不到「開多直」；而車子跑歪看起來很像 `drive_kD` 沒調好，其實是 `heading_kP` 沒調。
名字照第二節的規矩加機構前綴，全車唯一。

| 機構 | 滑桿 | 什麼時候被讀進 PID |
|---|---|---|
| 底盤直走 | `drive_kP` / `drive_kI` / `drive_kD` | **每次呼叫 `drive_distance()` 時**（PID 物件在函式開頭建構、複製增益） |
| 底盤循跡 | `heading_kP` / `heading_kI` / `heading_kD`（新增） | 同上，同一次呼叫裡的第二個 PID 物件 |
| 轉彎 | `turn_kP` / `turn_kI` / `turn_kD` | **每次呼叫 `turn_to_angle()` 時** |
| 手臂 | `arm_kP/kI/kD/kG`、`arm_horizontal_deg` | **每一圈**（第二節已修成每圈重抄） |
| 滑軌 | `cascade_kP/kI/kD/kG` | **每一圈**（第三節已修成每圈重抄） |

所以底盤那四組的操作順序是**先拉滑桿、再按鍵**：每按一次鍵＝重新讀一次滑桿現值，
動作跑到一半才拉滑桿不會影響那一次動作（要看階躍響應，這反而是對的行為）。
調參版**刻意不呼叫 `default_constants()`**——呼叫的話會把 dashboard 上剛調好的增益
覆蓋回程式碼裡的硬編值；`initialize()` 已經呼叫過一次，開機值本來就是對的。

**沒有開成滑桿的東西**：exit conditions（`drive_settle_error` / `drive_settle_time` /
`drive_timeout`，以及 turn 的那三個）刻意留在程式碼裡，理由是保持面板簡單、而且中止機制
依賴 `*_timeout` 不為 0（見 6.4 最後一段）。想調這幾個要改
`src/robot-config.cpp` 的 `default_constants()` 再重編。

### 6.5 比賽狀態下強制關閉

- 主迴圈每一圈都看 `pros::competition::is_disabled() || is_autonomous()`，成立就**不收任何按鍵**、
  螢幕顯示 `COMP LOCK`。
- 另外有一支**永遠在跑的看門狗 task**：場控在 disable／進入自走的瞬間會砍掉 `opcontrol`，
  但**不會**砍掉跑底盤動作的那支 worker task——它會若無其事繼續開車。看門狗每 20 ms 檢查一次，
  一離開遙控期就立刻把動作掐掉。
- 看門狗刻意**不碰手臂與滑軌**：自走期間那兩個是自走程式在管，而 `disabled()` 本來就已經
  把滑軌控制器停掉了（第一節做的）。它只負責一件事：**調參動作不准活過遙控期**。

### 6.6 調參流程建議

1. 車放在**空曠的地方**（前後至少留 1.5 公尺、左右留得下轉 180°），第一次有人守著電源。
2. 燒調參版到 slot 2，開機選 slot 2。**遙控器螢幕第一行會一直掛著 `== PID TUNE ==`、
   開場震兩下**——看到這個才是調參版。
3. dashboard 連上（`ws://192.168.4.1`），打開 Graph 面板。
4. **拉滑桿 → 按鍵實跑 → 看曲線**，一次只動一個增益：
   - 底盤直走「開多遠」：拉 `drive_kP` / `drive_kI` / `drive_kD` → 按 **L1**（前進 50 cm）→
     看 `drive_error` / `drive_target` / `drive_output` 三條線。回不去就按 **L2** 開回來，
     或直接用搖桿把車推回起點。
   - 底盤直走「開多直」：拉 `heading_kP` / `heading_kI` / `heading_kD`（新增，見 6.4a）→
     一樣按 **L1**／**L2**，看車尾有沒有偏。**車子跑歪先調這一組**，不是 `drive_kD`。
   - 轉彎：拉 `turn_kP` / `turn_kI` / `turn_kD` → 按 **R1**（左 90°）或 **R2**（右 180°）→
     看 `turn_error` / `turn_target` / `turn_output`。
   - 滑軌：拉 `cascade_kG` → `cascade_kP` → `cascade_kD` → 按 **A**／**B** →
     看 `cascade_pos` / `cascade_target` / `cascade_error` / `cascade_output` / `cascade_ff`。
     （順序照第三節第 4 點：先 kG，再 kP，再 kD，kI 最後。）
   - 手臂：拉 `arm_kG` → `arm_kP` → `arm_kD` → 按 **UP**／**DOWN** →
     看 `arm_angle` / `arm_target` / `arm_error` / `arm_output` / `arm_ff`。
5. **時序很重要：先拉滑桿，再按鍵。** 底盤那兩個函式是在被呼叫的那一瞬間才把
   `chassis.drive_kp/ki/kd`（`turn_*` 同理）複製進 PID 物件的，所以**每按一次鍵＝重新讀一次
   滑桿現值**；動作跑到一半才拉滑桿不會影響那一次動作（要看階躍響應，這反而是對的行為）。
   手臂與滑軌是每圈重抄（第二、三節已修），拉了立刻生效。
6. 調完的數字**一定要抄回程式碼再重燒正常版**——dashboard 上調的值斷電就沒了。
   - 底盤直走／循跡／轉彎：`src/robot-config.cpp` 的 `default_constants()`
     （分別是 `set_drive_constants` / `set_heading_constants` / `set_turn_constants`）
   - 手臂：`src/Template/arm.cpp` 上方
   - 滑軌：`src/Template/cascade.cpp` 上方
7. **抄完用 `pros make comp` 重編、燒 slot 1**，並確認比賽當天選的是 slot 1。
   **不可以直接 `pros mu --slot 1`**——那樣會把上一輪調參版的物件檔原封不動燒進比賽 slot
   （原因見 6.2 的警告框）。燒完開機確認 slot 1 的遙控器螢幕**沒有** `== PID TUNE ==`。

### 6.7 這一段動了哪些檔案

| 檔案 | 改了什麼 | 對正常（比賽）版的影響 |
|---|---|---|
| `src/tune_opcontrol.cpp`（新） | 整支調參程式 | **無**——整個檔案包在 `#ifdef PID_TUNE_PROGRAM` 裡，正常版編出來是空的 |
| `include/tune_opcontrol.h`（新） | 一行函式宣告＋說明 | 無（只是宣告） |
| `src/main.cpp` | ① `opcontrol()` 包成 `#ifdef` / `#else`（`#else` 那段是原本的內容，一個字沒改）② `initialize()` 多登記 `heading_kP/kI/kD` 三顆滑桿 | ①**零行為差** ②**兩版都生效**：多三顆滑桿，不改任何預設值 |
| `include/Template/arm.h`、`src/Template/arm.cpp` | 加了 `arm_hold_here()`（中止鍵用），**整段包在 `#ifdef PID_TUNE_PROGRAM` 裡** | **零行為差**（正常版根本沒編到這幾行） |
| `Makefile` | 新增 `tune` 與 `comp` 兩個 target | 無（`.DEFAULT_GOAL` 仍是 `quick`，已驗證 `make` 不帶旗標） |

**沒有動到**：`Drive::control_arcade()`（正常駕駛面一個字沒改）、`cascade.cpp`／`cascade.h`、
共用的 `PID` class、`drive.cpp` 的任何一行、`include/vexdash*`／`src/vexdash*` 整包。

### 6.8 駕駛面前後對照（正常版 vs 調參版）

| 鍵 | 正常版（slot 1，第二節那張表） | 調參版（slot 2） |
|---|---|---|
| L1 / L2 | 滑軌點動上／下 | 前進／後退 50 cm |
| R1 / R2 | intake 正／反轉 | 左轉 90°／右轉 180° |
| A | （未使用，只翻一個沒接東西的旗標） | 滑軌升到 595 |
| B | 爪子 toggle | 滑軌降到 0 |
| X | toggle 氣缸 | 無動作（中止鍵） |
| Y | （未使用） | 無動作（中止鍵） |
| UP | （已註解掉，無動作） | 手臂到 POS_2 (160°) |
| DOWN | 取消序列＋手臂回 DOWN | 手臂回 DOWN (0°) |
| LEFT | LEFT 預設序列 | 無動作（中止鍵） |
| RIGHT | RIGHT 預設序列 | 無動作（中止鍵） |
| 搖桿 | 方向盤式駕駛＋黏著煞車模式 | 方向盤式駕駛（沒有黏著煞車邏輯；測試動作跑的時候失效） |
| 限位開關歸零 | 有 | **沒有**（調參版沒有跑 `control_arcade()` 那段自癒；A／B 測試前記得先把滑軌降到底一次，開場的 tare 就是在那個位置歸零的） |

**正常版那一欄跟第二節那張表完全一致——這個 PR 的第六節沒有動它任何一格。**

### 6.9 這一節的風險與取捨

- 🔴 **最容易出事的一條：調參完直接 `pros mu --slot 1` 會把調參版燒進比賽 slot。**
  `tune` 只在進去的時候 clean、出來不會，所以跑完調參版之後 `bin/` 裡每個 `.o` 都帶著
  `-DPID_TUNE_PROGRAM`，而預設目標 `quick` 不 clean、make 又只看時間戳，於是判定「不用重編」。
  **回比賽版一定要 `pros make comp`（或先 `pros make clean`）**，燒完開機確認 slot 1
  的遙控器螢幕沒有 `== PID TUNE ==`。這是這一節唯一會直接害到比賽的坑，請寫進隊上的
  燒錄 checklist。
- **一樣沒有上車驗過**，也沒有真的跑過 `pros make`（沒有 ARM toolchain）。做的是主機端語法門
  ＋ Makefile 的 `-n` 乾跑，見下。
- **調參版沒有限位開關自癒**（見上表）。刻意的：那段邏輯長在 `control_arcade()` 裡面，
  要拿來用就得動正常駕駛面。代價是滑軌編碼器在長時間調參後可能會漂——重開一次程式就好。
- **中止底盤動作之後，最多要等 3 秒**那支動作才真的結束（輪子在第一個 10 ms 就已經 0 V 了，
  等的只是迴圈自己逾時）。這 3 秒內不收新的測試指令。這是為了不砍 task（見 6.4）付的代價。
- **調參版沒有自走保護以外的比賽適應**。它本來就不該上場：燒 slot 2、比賽選 slot 1。
- **手臂的 `arm_hold_here()` 是這個 PR 唯一動到 arm 控制器的地方**，而且整段包在
  `#ifdef` 裡。如果你們覺得連 `#ifdef` 都不想要，把它拿掉、中止鍵對手臂就不做事即可
  （手臂的動作本來就是既有 preset，跟正常駕駛按 DOWN 走的是同一條路）。
- **`heading_kP/kI/kD` 三顆新滑桿在正常版也會出現**（它們掛在 `initialize()`，不在 `#ifdef` 裡）。
  只是多三顆可以拉的滑桿，預設值完全沒動，不拉就跟以前一樣。
- **`pros make tune` / `pros make comp` 沒有真的跑完過**，只用 GNU Make 4.4.1 做過 `-n` 乾跑，確認：
  ① `tune` 會先 `clean` 再帶旗標重編；② `-DPID_TUNE_PROGRAM` 確實出現在每一行 `.cpp` 的
  編譯指令上；③ `comp` 會先 `clean` 再**不帶**旗標重編；④ 不帶 target 的 `make` 預設目標
  仍是 `quick`、指令列上沒有那個旗標。真正的 ARM 編譯還是要你們跑一次。

### 6.10 第六節做過的驗證

同第五節的主機端語法門，**兩個方向都跑**：

```
# 正常版
g++ -std=c++20 -D_USE_MATH_DEFINES -fsyntax-only -w -Iinclude -Iinclude/Template/pure-pursuit <每一個 src/**/*.cpp>
# 調參版
g++ -std=c++20 -D_USE_MATH_DEFINES -DPID_TUNE_PROGRAM -fsyntax-only -w -Iinclude -Iinclude/Template/pure-pursuit <同上>
```

| | 檔案數 | 通過 | 沒過 |
|---|---|---|---|
| 這個 PR 之前（第五節的基線） | 37 | 36 | `usb_serial_transport.cpp` |
| 正常版（無旗標） | **38** | **37** | `usb_serial_transport.cpp` |
| 調參版（`-DPID_TUNE_PROGRAM`） | **38** | **37** | `usb_serial_transport.cpp` |

唯一沒過的還是那個 POSIX `fcntl` 的老問題，**改動前後完全相同、跟這次無關**（見第五節）。
新增的 `tune_opcontrol.cpp` 另外用 `-Wall` 單獨編過，**零警告**。

真正的 `pros make tune` 與上車調參留給你們做。


🤖 Generated with [Claude Code](https://claude.com/claude-code)
