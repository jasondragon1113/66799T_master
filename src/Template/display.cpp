#include "main.h"
#include <cstring>
#include <cstdio>
#include <cmath>

uint32_t hsv_to_rgb(float h, float s, float v, float time) {
    int i = int(h / 60) % 6;
    float f = h / 60 - i;
    float p = v * (1 - s);
    float q = v * (1 - f * s);
    float t = v * (1 - (1 - f) * s);

    float r, g, b;
    switch (i) {
        case 0: r = v; g = t; b = p; break;
        case 1: r = q; g = v; b = p; break;
        case 2: r = p; g = v; b = t; break;
        case 3: r = p; g = q; b = v; break;
        case 4: r = t; g = p; b = v; break;
        default:r = v; g = p; b = q; break;
    }
    
    // time = time / 100000; // scale speed
    // r = uint8_t((sin(time) * 0.5f + 0.5f) * 255);
    // g = uint8_t((sin(time + 2) * 0.5f + 0.5f) * 255);
    // b = uint8_t((sin(time + 4) * 0.5f + 0.5f) * 255);

    return (uint32_t(r * 255) << 16) | (uint32_t(g * 255) << 8) | uint32_t(b * 255);
}

uint32_t rgb_to_uint32(int r, int g, int b) {
    return (uint32_t(r) << 16) | (uint32_t(g) << 8) | uint32_t(b);
}

int overheat = 0;
int screen_time = 0;
void simple_screen_task(bool rainbow){
	while (true){
		pros::screen::erase();
    int overheat_threshold = 45;
    overheat = 0;

    if(chassis.DriveL.get_temperature(0) > overheat_threshold || chassis.DriveL.get_temperature(1) > overheat_threshold || chassis.DriveL.get_temperature(2) > overheat_threshold){
      pros::screen::print(TEXT_LARGE_CENTER, 10, "DRIVE_L OVERHEATING");
      overheat = 1;
    }
    if(chassis.DriveR.get_temperature(0) > overheat_threshold || chassis.DriveR.get_temperature(1) > overheat_threshold || chassis.DriveR.get_temperature(2) > overheat_threshold){
      pros::screen::print(TEXT_LARGE_CENTER, 10, "DRIVE_R OVERHEATING");
      overheat = 1;
	  }
    
    uint32_t flash_color;

    switch(overheat){
      case 1:
        flash_color = pros::c::COLOR_RED;
        break;

      case 2:
        flash_color = pros::c::COLOR_ORANGE;
        break;
      
      case 3:
        flash_color = pros::c::COLOR_GREEN;
        break;
      
      default:
        // pros::screen::set_pen(uint32_t((screen_time * 10) % 16777215));
        // pros::screen::set_eraser(uint32_t((screen_time * 10) % 16777215));
        float hue = fmod(screen_time  * 0.06, 360);
        // pros::screen::set_eraser(hsv_to_rgb(hue, 1, 1, screen_time));
        pros::screen::set_pen(pros::c::COLOR_BLACK);
        pros::screen::set_eraser(pros::c::COLOR_BLACK);
        pros::screen::fill_rect(1,1,480,240);
        pros::screen::set_pen(rainbow? hsv_to_rgb(hue, 1, 1, screen_time) : pros::c::COLOR_WHITE);
        break;
    }

    if(overheat != 0) {
      if(screen_time % 250 < 125){
        pros::screen::set_pen(pros::c::COLOR_BLACK);
        pros::screen::set_eraser(pros::c::COLOR_BLACK);
        pros::screen::fill_rect(1,1,480,240);
      }
      else{
        pros::screen::set_pen(flash_color);
        pros::screen::set_eraser(flash_color);
        pros::screen::fill_rect(1,1,480,240);
      }
      pros::screen::set_pen(pros::c::COLOR_WHITE);
    }
    
		pros::screen::print(TEXT_MEDIUM, 0, "X: %.2f,  Y: %.2f, Heading: %.2f", chassis.get_X_position(), chassis.get_Y_position(), chassis.get_absolute_heading());
    pros::screen::print(TEXT_MEDIUM, 1, "Pitch: %.2f, Roll: %.2f", chassis.Gyro.get_pitch(), chassis.Gyro.get_roll());
    pros::screen::print(TEXT_MEDIUM, 2, "Drive L pos: %.2f, Drive R pos: %.2f", chassis.DriveL.get_position(), chassis.DriveR.get_position());
		pros::screen::print(TEXT_MEDIUM, 4, "DT L temp: %.2f, %.2f, %.2f", chassis.DriveL.get_temperature(0), chassis.DriveL.get_temperature(1), chassis.DriveL.get_temperature(2));
		pros::screen::print(TEXT_MEDIUM, 5, "DT R temp: %.2f, %.2f, %.2f", chassis.DriveR.get_temperature(0), chassis.DriveR.get_temperature(1), chassis.DriveR.get_temperature(2));
		pros::screen::print(TEXT_MEDIUM, 6, "DT L Torque (Nm): %.2f, %.2f, %.2f", chassis.DriveL.get_torque(0), chassis.DriveL.get_torque(1), chassis.DriveL.get_torque(2));
		pros::screen::print(TEXT_MEDIUM, 7, "DT R Torque (Nm): %.2f, %.2f, %.2f", chassis.DriveR.get_torque(0), chassis.DriveR.get_torque(1), chassis.DriveR.get_torque(2));

    // pros::screen::print(TEXT_LARGE_CENTER, 10, "progress: %d", progress);
    
    // float avg_torque = (chassis.DriveL.get_torque(0) + chassis.DriveL.get_torque(1) + chassis.DriveL.get_torque(2) + chassis.DriveR.get_torque(0) + chassis.DriveR.get_torque(1) + chassis.DriveR.get_torque(2)) / 6.0;
    // printf("avg_torque: %.2f\n", avg_torque);

    printf("roll: %.2f\n", chassis.Gyro.get_roll());

    delay(50);
    screen_time += 50;
  }
}

void print_curvepoints(std::vector<CurvePoint> points){
  for(int i = 0; i < points.size(); i++){
    print_point(points[i].point, i);
    // TODO: also print drive_voltage, heading_max_voltage, and follow_distance
    // TODO: add a way to edit drive_voltage, heading_max_voltage, and follow_distance for each point
  }
}

void print_point(Point point, int line){
  screen::print(TEXT_MEDIUM, line, "Point: x %.1f, y %.1f, h %.0f, has_h: %s", point.x, point.y, point.heading, point.has_heading ? "true" : "false");
}

// vex screen is 480*240, with (0, 0) in the top left
void init_map(){
  screen::set_pen(pros::c::COLOR_WHITE);
  screen::draw_rect(120, 0, 360, 239);
}

void map_curvepoints(std::vector<CurvePoint> points){
  for(int i = 0; i < points.size(); i++){
    // game manual version
    int x = points[i].point.x / 140.41 * 120 + 120;
    int y = 120 - points[i].point.y / 140.41 * 120;

    // path.jerryio version
    // int x = points[i].point.x / 70 * 60 + 239;
    // int y = 60 - points[i].point.y / 70 * 60;

    screen::set_pen(pros::c::COLOR_RED);
    screen::draw_pixel(x, y);
  }
}

void update_map(){
  // game manual version
  int robot_x = chassis.get_X_position() / 140.41 * 120 + 120;
  int robot_y = 120 - chassis.get_Y_position() / 140.41 * 120;

  // path.jerryio version
  // int robot_x = chassis.get_X_position() / 70 * 60 + 239;
  // int robot_y = 60 - chassis.get_Y_position() / 70 * 60;
  
  screen::set_pen(pros::c::COLOR_YELLOW_GREEN);
  screen::draw_pixel(robot_x, robot_y);

  // printf("last_found_index: %d, L_voltage: %d, R_voltage: %dp\n", chassis.last_found_index, leftMotors.get_voltage(), rightMotors.get_voltage());
}

void map_task(){
  init_map();
  while(true){
    update_map();
    delay(50);
  }
}

// ============================================================================
// Dashboard: 3-tab touchscreen UI (Motors / Position / Auton Select)
// SAO tab disabled, see sao_gallery.cpp
// Screen is 480x240. Tab bar occupies y 0-26, content area is y 26-240.
// ============================================================================

AutonRoutine selected_auton = AutonRoutine::left;

DisplayTab current_tab = DisplayTab::MOTORS;

const int TAB_BAR_HEIGHT = 26;
const int TAB_WIDTH = 120;
const int OVERHEAT_THRESHOLD = 45;

// Color palette (kept in one place so the whole UI stays visually consistent)
const uint32_t COLOR_BG      = pros::c::COLOR_BLACK;
const uint32_t COLOR_SURFACE = pros::c::COLOR_BLACK;
const uint32_t COLOR_BORDER  = pros::c::COLOR_SILVER;
const uint32_t COLOR_TEXT    = pros::c::COLOR_WHITE_SMOKE;
const uint32_t COLOR_ACCENT  = pros::c::COLOR_DODGER_BLUE;
const uint32_t COLOR_SHADOW  = pros::c::COLOR_DIM_GRAY;

// Draws a filled rectangle with rounded corners using the classic
// "cross of rects + 4 corner circles" trick (no native rounded-rect API).
void draw_rounded_rect_filled(int x0, int y0, int x1, int y1, int r){
  screen::fill_rect(x0 + r, y0, x1 - r, y1);
  screen::fill_rect(x0, y0 + r, x1, y1 - r);
  screen::fill_circle(x0 + r, y0 + r, r);
  screen::fill_circle(x1 - r, y0 + r, r);
  screen::fill_circle(x0 + r, y1 - r, r);
  screen::fill_circle(x1 - r, y1 - r, r);
}

// Draws a quarter-circle arc (in 3-degree steps) since the screen API has no arc primitive.
void draw_quarter_arc(int cx, int cy, int r, int start_deg, int end_deg){
  for(int deg = start_deg; deg <= end_deg; deg += 3){
    double rad = deg * 3.14159265358979 / 180.0;
    int x = cx + (int)(r * std::cos(rad));
    int y = cy + (int)(r * std::sin(rad));
    screen::draw_pixel(x, y);
  }
}

// Outline-only rounded rectangle: 4 straight edges + 4 corner arcs.
void draw_rounded_rect_outline(int x0, int y0, int x1, int y1, int r){
  screen::draw_line(x0 + r, y0, x1 - r, y0);
  screen::draw_line(x0 + r, y1, x1 - r, y1);
  screen::draw_line(x0, y0 + r, x0, y1 - r);
  screen::draw_line(x1, y0 + r, x1, y1 - r);
  draw_quarter_arc(x0 + r, y0 + r, r, 180, 270);
  draw_quarter_arc(x1 - r, y0 + r, r, 270, 360);
  draw_quarter_arc(x0 + r, y1 - r, r, 90, 180);
  draw_quarter_arc(x1 - r, y1 - r, r, 0, 90);
}

struct MotorInfo {
  const char* name;
  Motor* motor;
};

// leftFront/leftMiddle/.../cascade2/arm are declared in robot-config.h
MotorInfo dashboard_motors[10] = {
  {"leftFront",   &leftFront},
  {"leftMiddle",  &leftMiddle},
  {"leftBack",    &leftBack},
  {"rightFront",  &rightFront},
  {"rightMiddle", &rightMiddle},
  {"rightBack",   &rightBack},
  {"intake",      &intake},
  {"cascade1",    &cascade1},
  {"cascade2",    &cascade2},
  {"arm",         &arm},
};

// Auton Select box geometry, shared between drawing and touch hit-testing.
const int AUTON_BOX_W = 120;
const int AUTON_BOX_H = 65;
const int AUTON_BOX_Y = 45;
const int AUTON_GAP = 20;
const int AUTON_START_X = (480 - (AUTON_BOX_W * 3 + AUTON_GAP * 2)) / 2;

// The (x,y) print_at overload always treats (x,y) as the TOP-LEFT corner of
// the text, even for the "_CENTER" format variants -- they do NOT auto-center
// around the given point. So to actually center or right-pad text we have to
// estimate its rendered width ourselves and compute the left x manually.
int dashboard_text_width(pros::text_format_e_t fmt, const char* text){
  int char_w = (fmt == TEXT_LARGE) ? 20 : 8;
  return (int)strlen(text) * char_w;
}

int dashboard_center_x(pros::text_format_e_t fmt, int x0, int x1, const char* text){
  int w = dashboard_text_width(fmt, text);
  int x = x0 + ((x1 - x0) - w) / 2;
  if(x < x0) x = x0;
  return x;
}

void dashboard_draw_tab_bar(){
  const char* labels[3] = {"MOTORS", "POSITION", "AUTON"}; // SAO tab disabled, see sao_gallery.cpp
  const int margin = 4;
  const int radius = 8;

  screen::set_pen(COLOR_BG);
  screen::set_eraser(COLOR_BG);
  screen::fill_rect(0, 0, 480, TAB_BAR_HEIGHT + 2);

  for(int i = 0; i < 3; i++){
    int slot_x0 = i * TAB_WIDTH;
    int slot_x1 = slot_x0 + TAB_WIDTH;
    int x0 = slot_x0 + margin;
    int x1 = slot_x1 - margin;
    int text_x = dashboard_center_x(TEXT_MEDIUM, x0, x1, labels[i]) - 7;
    if(text_x < x0) text_x = x0;
    int text_y = TAB_BAR_HEIGHT / 2 - 8;

    if((int)current_tab == i){
      screen::set_pen(COLOR_ACCENT);
      screen::set_eraser(COLOR_ACCENT);
      draw_rounded_rect_filled(x0, 1, x1, TAB_BAR_HEIGHT, radius);
      screen::set_pen(pros::c::COLOR_WHITE);
      screen::print(TEXT_MEDIUM, text_x, text_y, labels[i]);
    } else {
      screen::set_pen(COLOR_BORDER);
      screen::set_eraser(COLOR_BG);
      draw_rounded_rect_outline(x0, 1, x1, TAB_BAR_HEIGHT, radius);
      screen::set_pen(COLOR_TEXT);
      screen::print(TEXT_MEDIUM, text_x, text_y, labels[i]);
    }
  }
}

// Motors tab + IMU calibrate button geometry, shared with touch hit-testing.
const int MOTOR_START_Y = 38;
const int MOTOR_ROW_H = 28;
const int MOTOR_COL0_ROWS = 5;
const int IMU_FOOTER_Y = MOTOR_START_Y + MOTOR_COL0_ROWS * MOTOR_ROW_H + 4;
const int IMU_ROW_Y = IMU_FOOTER_Y + 28;
const int IMU_BTN_X0 = 340;
const int IMU_BTN_X1 = 460;
const int IMU_BTN_Y0 = IMU_ROW_Y - 4;
const int IMU_BTN_Y1 = IMU_BTN_Y0 + 26;

void dashboard_draw_motors_tab(){
  int col_x[2] = {10, 250};
  int col_counts[2] = {5, 5};
  int row_h = MOTOR_ROW_H;
  int start_y = MOTOR_START_Y;
  int idx = 0;

  screen::set_eraser(pros::c::COLOR_BLACK);

  for(int col = 0; col < 2; col++){
    for(int row = 0; row < col_counts[col]; row++){
      MotorInfo& info = dashboard_motors[idx];
      int x = col_x[col];
      int y = start_y + row * row_h;

      double temp = info.motor->get_temperature();
      bool disconnected = std::isinf(temp) || std::isnan(temp);

      uint32_t status_color;
      if(disconnected){
        status_color = pros::c::COLOR_RED;
      } else if(temp > OVERHEAT_THRESHOLD){
        status_color = pros::c::COLOR_YELLOW;
      } else {
        status_color = pros::c::COLOR_GREEN;
      }

      screen::set_pen(status_color);
      screen::fill_circle(x + 5, y + 8, 5);

      // Clear just this row's text field so stale characters from a
      // previous (longer) value can't linger -- cheaper and less flashy
      // than blanking the whole tab every cycle.
      screen::set_pen(COLOR_BG);
      screen::set_eraser(COLOR_BG);
      screen::fill_rect(x + 20, y, x + 230, y + row_h - 2);

      screen::set_pen(COLOR_TEXT);
      if(disconnected){
        screen::print(TEXT_MEDIUM, x + 20, y, "%-12s NA", info.name);
      } else {
        double power = info.motor->get_power();
        screen::print(TEXT_MEDIUM, x + 20, y, "%-12s %.0fC %.1fW", info.name, temp, power);
      }

      idx++;
    }
  }

  // Sensors footer (IMU, plus tracking rotation sensors)
  screen::set_pen(COLOR_BORDER);
  screen::print(TEXT_MEDIUM, 10, IMU_FOOTER_Y, "SENSORS");

  bool imu_installed = inertial.is_installed();
  pros::ImuStatus imu_status = inertial.get_status();
  bool imu_calibrating = imu_status == pros::ImuStatus::calibrating;
  uint32_t imu_color;
  const char* imu_text;
  if(!imu_installed || imu_status == pros::ImuStatus::error){
    imu_color = pros::c::COLOR_RED;
  } else if(imu_calibrating){
    imu_color = pros::c::COLOR_YELLOW;
  } else {
    imu_color = pros::c::COLOR_GREEN;
  }
  imu_text = "IMU"; // status conveyed by the dot color, same as the L/R distance labels below

  screen::set_pen(COLOR_BG);
  screen::set_eraser(COLOR_BG);
  screen::fill_rect(30, IMU_ROW_Y, 185, IMU_ROW_Y + row_h - 2);
  screen::set_pen(imu_color);
  screen::fill_circle(15, IMU_ROW_Y + 8, 5);
  screen::set_pen(COLOR_TEXT);
  screen::print(TEXT_MEDIUM, 30, IMU_ROW_Y, imu_text);

  // Distance sensors L/R, same row as IMU status.
  char dist_buf[16];

  screen::set_pen(COLOR_BG);
  screen::set_eraser(COLOR_BG);
  screen::fill_rect(200, IMU_ROW_Y, 260, IMU_ROW_Y + row_h - 2);
  bool dist_l_installed = distance_sensorL.is_installed();
  screen::set_pen(dist_l_installed ? pros::c::COLOR_GREEN : pros::c::COLOR_RED);
  screen::fill_circle(190, IMU_ROW_Y + 8, 5);
  screen::set_pen(COLOR_TEXT);
  if(dist_l_installed){
    snprintf(dist_buf, sizeof(dist_buf), "L %dmm", (int)distance_sensorL.get());
  } else {
    snprintf(dist_buf, sizeof(dist_buf), "L NA");
  }
  screen::print(TEXT_MEDIUM, 200, IMU_ROW_Y, "%s", dist_buf);

  screen::set_pen(COLOR_BG);
  screen::set_eraser(COLOR_BG);
  screen::fill_rect(275, IMU_ROW_Y, 335, IMU_ROW_Y + row_h - 2);
  bool dist_r_installed = distance_sensorR.is_installed();
  screen::set_pen(dist_r_installed ? pros::c::COLOR_GREEN : pros::c::COLOR_RED);
  screen::fill_circle(265, IMU_ROW_Y + 8, 5);
  screen::set_pen(COLOR_TEXT);
  if(dist_r_installed){
    snprintf(dist_buf, sizeof(dist_buf), "R %dmm", (int)distance_sensorR.get());
  } else {
    snprintf(dist_buf, sizeof(dist_buf), "R NA");
  }
  screen::print(TEXT_MEDIUM, 275, IMU_ROW_Y, "%s", dist_buf);

  // Button to (re)initialize/calibrate the IMU. Interior is cleared first --
  // draw_rounded_rect_outline only paints the border, so without this a
  // previous filled state (e.g. "CALIBRATING") would leave stale fill behind.
  screen::set_pen(COLOR_BG);
  screen::set_eraser(COLOR_BG);
  screen::fill_rect(IMU_BTN_X0, IMU_BTN_Y0, IMU_BTN_X1, IMU_BTN_Y1);

  const char* btn_text;
  if(!imu_installed){
    // Disabled look: no IMU to calibrate, so the button shouldn't read as actionable.
    screen::set_pen(COLOR_SHADOW);
    screen::set_eraser(COLOR_BG);
    draw_rounded_rect_outline(IMU_BTN_X0, IMU_BTN_Y0, IMU_BTN_X1, IMU_BTN_Y1, 8);
    screen::set_pen(COLOR_SHADOW);
    btn_text = "NO IMU";
  } else if(imu_calibrating){
    // Filled with the same yellow as the status dot, so "in progress" reads consistently.
    screen::set_pen(imu_color);
    screen::set_eraser(imu_color);
    draw_rounded_rect_filled(IMU_BTN_X0, IMU_BTN_Y0, IMU_BTN_X1, IMU_BTN_Y1, 8);
    screen::set_pen(pros::c::COLOR_BLACK);
    btn_text = "CALIBRATING";
  } else {
    screen::set_pen(COLOR_BORDER);
    screen::set_eraser(COLOR_BG);
    draw_rounded_rect_outline(IMU_BTN_X0, IMU_BTN_Y0, IMU_BTN_X1, IMU_BTN_Y1, 8);
    screen::set_pen(COLOR_TEXT);
    btn_text = "CALIBRATE";
  }
  screen::print(TEXT_MEDIUM, IMU_BTN_X0 + 8, IMU_BTN_Y0 + 4, "%s", btn_text);
}

void dashboard_draw_position_tab(){
  char buf[48];

  // Clear the whole text block before redrawing (one generous rect, so a
  // taller-than-expected line can't get clipped and vanish) so stale
  // characters from a previous (longer) value can't linger either.
  screen::set_pen(COLOR_BG);
  screen::set_eraser(COLOR_BG);
  screen::fill_rect(15, 55, 465, 200);
  screen::set_pen(COLOR_TEXT);
  screen::set_eraser(COLOR_BG);

  snprintf(buf, sizeof(buf), "X: %.2f\"      Y: %.2f\"", chassis.get_X_position(), chassis.get_Y_position());
  screen::print(TEXT_LARGE, 20, 60, "%s", buf);

  snprintf(buf, sizeof(buf), "Heading: %.1f deg", chassis.get_absolute_heading());
  screen::print(TEXT_LARGE, 20, 110, "%s", buf);

  snprintf(buf, sizeof(buf), "Pitch: %.2f   Roll: %.2f", chassis.Gyro.get_pitch(), chassis.Gyro.get_roll());
  screen::print(TEXT_MEDIUM, 20, 170, "%s", buf);
}

void dashboard_draw_auton_tab(){
  const char* labels[3] = {"left", "right", "sawp"};
  const int label_pad = 8;
  const int radius = 10;
  const int shadow_offset = 3;

  for(int i = 0; i < 3; i++){
    int x0 = AUTON_START_X + i * (AUTON_BOX_W + AUTON_GAP);
    int x1 = x0 + AUTON_BOX_W;
    int label_y = AUTON_BOX_Y + AUTON_BOX_H / 2 - 8;

    // Clear the box's full bounding area (including the shadow a selected
    // box draws) first -- an outline redraw only draws border lines, so
    // without this the old fill/shadow from a previous selection lingers.
    screen::set_pen(COLOR_BG);
    screen::set_eraser(COLOR_BG);
    screen::fill_rect(x0 - 1, AUTON_BOX_Y - 1, x1 + shadow_offset + 1, AUTON_BOX_Y + AUTON_BOX_H + shadow_offset + 1);

    if((int)selected_auton == i){
      screen::set_pen(COLOR_SHADOW);
      screen::set_eraser(COLOR_SHADOW);
      draw_rounded_rect_filled(x0 + shadow_offset, AUTON_BOX_Y + shadow_offset, x1 + shadow_offset, AUTON_BOX_Y + AUTON_BOX_H + shadow_offset, radius);

      screen::set_pen(COLOR_ACCENT);
      screen::set_eraser(COLOR_ACCENT);
      draw_rounded_rect_filled(x0, AUTON_BOX_Y, x1, AUTON_BOX_Y + AUTON_BOX_H, radius);
      screen::set_pen(pros::c::COLOR_WHITE);
      screen::print(TEXT_MEDIUM, x0 + label_pad, label_y, labels[i]);
    } else {
      screen::set_pen(COLOR_BORDER);
      screen::set_eraser(COLOR_BG);
      draw_rounded_rect_outline(x0, AUTON_BOX_Y, x1, AUTON_BOX_Y + AUTON_BOX_H, radius);
      screen::set_pen(COLOR_TEXT);
      screen::print(TEXT_MEDIUM, x0 + label_pad, label_y, labels[i]);
    }
  }

  char buf[32];
  snprintf(buf, sizeof(buf), "Selected: %s", labels[(int)selected_auton]);
  int selected_line_y = AUTON_BOX_Y + AUTON_BOX_H + 25;
  // Clear this line first -- a shorter label (e.g. "right" -> "left")
  // wouldn't otherwise overwrite the previous text's trailing characters.
  screen::set_pen(COLOR_BG);
  screen::set_eraser(COLOR_BG);
  screen::fill_rect(15, selected_line_y - 2, 300, selected_line_y + 18);
  screen::set_pen(COLOR_TEXT);
  screen::set_eraser(COLOR_BG);
  screen::print(TEXT_MEDIUM, 20, selected_line_y, "%s", buf);
}

// SAO tab: a personalization image gallery you flip through with Back/Next
// (SAO = Signature Autonomous Object, the customary VRC robot decoration).
// Disabled -- see include/Template/display.h and src/Template/sao_gallery.cpp.
// int sao_index = 0;
//
// const int SAO_IMAGE_Y = TAB_BAR_HEIGHT;
// const int SAO_BTN_Y0 = SAO_IMAGE_Y + SAO_IMAGE_H + 4;
// const int SAO_BTN_Y1 = 240 - 2;
// const int SAO_BTN_W = 120;
// const int SAO_BACK_X0 = 20;
// const int SAO_BACK_X1 = SAO_BACK_X0 + SAO_BTN_W;
// const int SAO_NEXT_X1 = 460;
// const int SAO_NEXT_X0 = SAO_NEXT_X1 - SAO_BTN_W;
//
// void dashboard_draw_sao_tab(){
//   screen::copy_area(0, SAO_IMAGE_Y, SAO_IMAGE_W - 1, SAO_IMAGE_Y + SAO_IMAGE_H - 1,
//                      const_cast<uint32_t*>(sao_images[sao_index]), SAO_IMAGE_W);
//
//   screen::set_pen(COLOR_BORDER);
//   screen::set_eraser(COLOR_BG);
//   draw_rounded_rect_outline(SAO_BACK_X0, SAO_BTN_Y0, SAO_BACK_X1, SAO_BTN_Y1, 8);
//   draw_rounded_rect_outline(SAO_NEXT_X0, SAO_BTN_Y0, SAO_NEXT_X1, SAO_BTN_Y1, 8);
//   screen::set_pen(COLOR_TEXT);
//   screen::print(TEXT_MEDIUM, SAO_BACK_X0 + 30, SAO_BTN_Y0 + 5, "BACK");
//   screen::print(TEXT_MEDIUM, SAO_NEXT_X0 + 30, SAO_BTN_Y0 + 5, "NEXT");
//   screen::print(TEXT_MEDIUM, 220, SAO_BTN_Y0 + 5, "%2d/%2d", sao_index + 1, SAO_IMAGE_COUNT);
// }

// Handles touch: switches tabs, and on the Auton Select tab, selects a routine.
// Only reacts to the press transition (not held/repeat) so one tap = one action.
void dashboard_handle_touch(){
  static last_touch_e_t prev_status = E_TOUCH_RELEASED;

  screen_touch_status_s_t touch = screen::touch_status();

  bool just_pressed = (touch.touch_status == E_TOUCH_PRESSED) && (prev_status != E_TOUCH_PRESSED);
  prev_status = touch.touch_status;

  if(!just_pressed) return;

  if(touch.y < TAB_BAR_HEIGHT){
    int tab_index = touch.x / TAB_WIDTH;
    if(tab_index >= 0 && tab_index <= 2){ // SAO tab disabled, see sao_gallery.cpp
      current_tab = (DisplayTab)tab_index;
    }
    return;
  }

  if(current_tab == DisplayTab::MOTORS){
    if(touch.x >= IMU_BTN_X0 && touch.x <= IMU_BTN_X1 && touch.y >= IMU_BTN_Y0 && touch.y <= IMU_BTN_Y1){
      if(inertial.is_installed() && !inertial.is_calibrating()){
        inertial.reset(false); // non-blocking: status flips to "calibrating" and the tab reflects it live
      }
    }
    return;
  }

  if(current_tab == DisplayTab::AUTON_SELECT){
    for(int i = 0; i < 3; i++){
      int x0 = AUTON_START_X + i * (AUTON_BOX_W + AUTON_GAP);
      int x1 = x0 + AUTON_BOX_W;
      if(touch.x >= x0 && touch.x <= x1 && touch.y >= AUTON_BOX_Y && touch.y <= AUTON_BOX_Y + AUTON_BOX_H){
        selected_auton = (AutonRoutine)i;
        break;
      }
    }
    return;
  }

  // SAO tab disabled, see sao_gallery.cpp
  // if(current_tab == DisplayTab::SAO){
  //   if(touch.y >= SAO_BTN_Y0 && touch.y <= SAO_BTN_Y1){
  //     if(touch.x >= SAO_BACK_X0 && touch.x <= SAO_BACK_X1){
  //       sao_index = (sao_index - 1 + SAO_IMAGE_COUNT) % SAO_IMAGE_COUNT;
  //     } else if(touch.x >= SAO_NEXT_X0 && touch.x <= SAO_NEXT_X1){
  //       sao_index = (sao_index + 1) % SAO_IMAGE_COUNT;
  //     }
  //   }
  // }
}

// Redraws only what actually changed instead of blanking the whole content
// area every loop -- clearing to black and immediately redrawing over it
// each cycle is what caused the visible screen flashing.
//
// Touch is polled every 25ms (its own cheap read) so taps register quickly;
// actual screen redrawing -- the expensive part -- is throttled separately
// so a slow frame (e.g. the SAO image copy) can't delay the next touch read
// and make the UI feel like it's dropping taps.
void dashboard_task(){
  DisplayTab prev_tab = (DisplayTab)-1;
  AutonRoutine prev_auton = (AutonRoutine)-1;
  // int prev_sao_index = -1; // SAO tab disabled, see sao_gallery.cpp
  int frame = 0;

  while(true){
    dashboard_handle_touch();

    bool tab_changed = current_tab != prev_tab;
    bool live_tick = tab_changed || (frame % 3 == 0); // ~75ms cadence for live tabs

    if(tab_changed){
      screen::set_pen(COLOR_BG);
      screen::set_eraser(COLOR_BG);
      screen::fill_rect(0, TAB_BAR_HEIGHT, 480, 240);
      dashboard_draw_tab_bar();
    }

    switch(current_tab){
      case DisplayTab::MOTORS:
        if(live_tick) dashboard_draw_motors_tab();
        break;
      case DisplayTab::POSITION:
        if(live_tick) dashboard_draw_position_tab();
        break;
      case DisplayTab::AUTON_SELECT:
        if(tab_changed || selected_auton != prev_auton){
          dashboard_draw_auton_tab();
        }
        break;
      // SAO tab disabled, see sao_gallery.cpp
      // case DisplayTab::SAO:
      //   if(tab_changed || sao_index != prev_sao_index){
      //     dashboard_draw_sao_tab();
      //   }
      //   break;
    }

    prev_tab = current_tab;
    prev_auton = selected_auton;
    // prev_sao_index = sao_index; // SAO tab disabled, see sao_gallery.cpp
    frame++;

    delay(25);
  }
}

// Safe to call from both initialize() and competition_initialize(): the
// underlying Task is a function-local static, so only the first call
// actually starts it.
void start_dashboard(){
  static Task screen_task(dashboard_task);
}