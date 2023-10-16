// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/command_FlashLEDs.h"

command_FlashLEDs::command_FlashLEDs(subsystem_LED* LED, units::time::second_t seconds) : m_LED{LED}, m_seconds{seconds}{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_LED});
}

// Called when the command is initially scheduled.
void command_FlashLEDs::Initialize() {
 // m_LED->ToggleFlashLEDs();
}

// Called repeatedly when this Command is scheduled to run
void command_FlashLEDs::Execute() {}

// Called once the command ends or is interrupted.
void command_FlashLEDs::End(bool interrupted) {
  // m_LED->ToggleFlashLEDs();
  // m_LED->SetStandbyLED();
}

// Returns true when the command should end.
bool command_FlashLEDs::IsFinished() {
  return m_Timer.Get() > m_seconds;
}
