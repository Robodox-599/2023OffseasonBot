// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "constants.h"
#include "frc/Timer.h"
#include <frc2/command/CommandPtr.h>
#include <ctre/phoenix/led/CANdle.h>
#include "ctre/phoenix/led/ColorFlowAnimation.h"
#include "ctre/phoenix/led/SingleFadeAnimation.h"
#include "ctre/phoenix/led/StrobeAnimation.h"


class subsystem_LED : public frc2::SubsystemBase {
 public:
  subsystem_LED();
  void SetPurpleLED();
  void SetYellowLED();
  void SetStandbyLED();
  void SetErrorLED();
  void SetOffLED();
  void SetTriggerStateLED();
  void ToggleFlashLEDs();
  LEDConstants::LEDState GetLEDState();
  void cycleLED();
  frc2::CommandPtr cycleLEDCommand();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  bool flashLEDs = false;
  ctre::phoenix::led::SingleFadeAnimation IntakedAnimation{255, 0, 0, 0, 0.5};
  ctre::phoenix::led::CANdle m_CANdle;
  frc::Timer m_LEDTimer;
  LEDConstants::LEDState m_LEDState = LEDConstants::LEDState::Standby;
  LEDConstants::LEDState m_TriggerState = LEDConstants::LEDState::Standby;
};