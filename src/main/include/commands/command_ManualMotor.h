// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "subsystems/subsystem_WristMotor.h"
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class command_ManualMotor
    : public frc2::CommandHelper<frc2::CommandBase, command_ManualMotor> {
 public:
  command_ManualMotor(subsystem_WristMotor* Wrist, std::function<double()> StickPos);
  

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
private:
std::function<double()> m_StickPos;
subsystem_WristMotor* m_Wrist;
};
