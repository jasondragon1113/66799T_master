#pragma once
#include "Template/arm.h"

// Cascade target, in cascade motor degrees, for each scoring level.
enum class ScoringLevel {
  LEVEL_0 = 0,
  LEVEL_1 = 300,
  LEVEL_2 = 1000,
  LEVEL_3 = 2000,
  LEVEL_4 = 3800
};

// Drives the cascade to `level`'s degree target and the arm to `arm_pos`
// (ArmPosition::DOWN = 0, POS_2 = 665, POS_1 = 1160 -- see arm.h), and blocks
// until both have actually arrived (or CASCADE_SCORE_TIMEOUT_MS elapses).
void score(ScoringLevel level, ArmPosition arm_pos);

void left();
void right();
void sawp();
