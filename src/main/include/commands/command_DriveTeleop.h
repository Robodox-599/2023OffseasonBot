// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/subsystem_DriveTank.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class command_DriveTeleop
    : public frc2::CommandHelper<frc2::CommandBase, command_DriveTeleop> {
 public:
  command_DriveTeleop(subsystem_DriveTank* Drive, std::function<double()> TurnStick, std::function<double()> DriveStick);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  subsystem_DriveTank* m_Drive;
  std::function<double()> m_TurnInput;
  std::function<double()> m_DriveInput;

};
