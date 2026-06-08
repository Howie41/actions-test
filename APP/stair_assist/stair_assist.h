#pragma once

#include <cstdint>

enum class StairAssistLaser1State : uint8_t {
  Invalid = 0,
  Far,
  NearStair,
  EdgeOpen,
};

enum class StairAssistLaser2State : uint8_t {
  Invalid = 0,
  GroundNormal,
  HighSuspended,
  StepContact,
};

struct StairAssistDebug {
  bool enabled{false};
  bool laser1_fresh{false};
  bool laser2_fresh{false};

  int32_t laser1_mm{0};
  int32_t laser2_mm{0};

  uint32_t laser1_frame_count{0};
  uint32_t laser2_frame_count{0};

  uint8_t laser1_state{static_cast<uint8_t>(StairAssistLaser1State::Invalid)};
  uint8_t laser2_state{static_cast<uint8_t>(StairAssistLaser2State::Invalid)};

  uint8_t laser1_near_count{0};
  uint8_t laser1_edge_count{0};
  uint8_t laser2_ground_count{0};
  uint8_t laser2_high_count{0};
  uint8_t laser2_step_count{0};

  bool saw_laser2_high_for_climb{false};
  bool saw_laser2_close_for_descend{false};

  bool suggest_climb_up{false};
  bool suggest_descend_edge_ready{false};
  bool should_lower_after_climb{false};
  bool should_lower_after_descend{false};
};

void stairAssistInit();
void stairAssistSetEnabled(bool enabled);
bool stairAssistEnabled();
void stairAssistUpdate();
void stairAssistResetProgress();

StairAssistLaser1State stairAssistLaser1State();
StairAssistLaser2State stairAssistLaser2State();

bool stairAssistSuggestClimbUp();
bool stairAssistSuggestDescendEdgeReady();
bool stairAssistShouldLowerAfterClimbAdvance();
bool stairAssistShouldLowerAfterDescendRetreat();

const StairAssistDebug &stairAssistDebug();
