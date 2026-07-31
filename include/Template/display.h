#pragma once
#include "main.h"
using namespace pros;

void simple_screen_task(bool rainbow);

void print_curvepoints(std::vector<CurvePoint> points);
void print_point(Point point, int line);

void init_map();
void map_curvepoints(std::vector<CurvePoint> points);
void update_map();
void map_task();

// Dashboard: motor temps/connection, robot coordinates, autonomous selector.

enum class DisplayTab {
    MOTORS = 0,
    POSITION = 1,
    AUTON_SELECT = 2,
    // SAO = 3, // SAO tab disabled, see sao_gallery.cpp
};

enum class AutonRoutine {
    left = 0,
    right = 1,
    sawp = 2
};

extern AutonRoutine selected_auton;

void dashboard_task();
void start_dashboard();
void dashboard_handle_touch();
// --- odometry pose telemetry (defined in display.cpp, published every 25 ms) ---
// JAR units: inches / degrees. Registered as graph channels under "drive/pid" in
// main.cpp; the Field panel's marker is fed separately by set_pose().
// 中文：里程計座標遙測（定義在 display.cpp，每 25ms 更新一次）。單位照 JAR：吋與度。
// 在 main.cpp 以 "drive/pid" 登記成圖表頻道；場地面板的圖示則是由 set_pose() 另外餵。
extern float tele_pose_x;
extern float tele_pose_y;
extern float tele_pose_heading;

void dashboard_draw_tab_bar();
void dashboard_draw_motors_tab();
void dashboard_draw_position_tab();
void dashboard_draw_auton_tab();
// void dashboard_draw_sao_tab(); // SAO tab disabled, see sao_gallery.cpp

// SAO gallery: desktop screenshots you can flip through with Back/Next (see sao_gallery.cpp).
// Disabled -- sao_gallery.cpp body is wrapped in #if 0.
// extern const int SAO_IMAGE_W;
// extern const int SAO_IMAGE_H;
// extern const int SAO_IMAGE_COUNT;
// extern const uint32_t* const sao_images[];