// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>
#include "subsystems/subsystem_LEDs.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class command_FlashLEDs
    : public frc2::CommandHelper<frc2::CommandBase, command_FlashLEDs> {
 public:
  command_FlashLEDs(subsystem_LED* LED, units::time::second_t seconds);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private: 
  frc::Timer m_Timer{};
  subsystem_LED* m_LED;
  units::time::second_t m_seconds;
};
