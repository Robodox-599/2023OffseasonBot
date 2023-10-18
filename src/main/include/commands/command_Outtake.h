// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>
#include "subsystems/subsystem_Intake.h"
#include "subsystems/subsystem_LEDs.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class command_Outtake
    : public frc2::CommandHelper<frc2::CommandBase, command_Outtake> {
 public:
  command_Outtake(subsystem_Intake* intake, subsystem_LED* LED);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
  private:
  subsystem_Intake* m_Intake;
  frc::Timer m_Timer{};
  subsystem_LED* m_LED;
};
