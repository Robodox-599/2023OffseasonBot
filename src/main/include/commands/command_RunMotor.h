// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/subsystem_WristMotor.h"
#include <frc/Timer.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class command_RunMotor
    : public frc2::CommandHelper<frc2::CommandBase, command_RunMotor> {
 public:
  command_RunMotor(subsystem_WristMotor* Wrist, 
                  std::function<double()> EncPosition, 
                  std::function<bool()> IsWait, 
                  std::function<double()> Threshold,
                  std::function<double()> StickPos); //unnessary im just too lazy to change it everywhre

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private: 
   subsystem_WristMotor* m_Wrist;
   std::function<double()> m_EncPosition;
   std::function<bool()> m_IsWait;
   std::function<double()> m_Threshold;
   std::function<double()> m_StickPos;
   frc::Timer m_Timer{};
};