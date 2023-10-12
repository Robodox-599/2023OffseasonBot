// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here
  frc2::JoystickButton(&XboxYaperator, 
                       frc::XboxController::Button::kA)
                       .OnTrue(m_LED.cycleLEDCommand());
  frc2::JoystickButton(&XboxYaperator, 
                       frc::XboxController::Button::kB)
                       .OnTrue(frc2::cmd::Sequence(command_Intake(&m_Intake, &m_LED).ToPtr(),
                                                    command_FlashLEDs(&m_LED, LEDConstants::IntakeFlashTime).ToPtr()));
  frc2::JoystickButton(&XboxYaperator, 
                       frc::XboxController::Button::kX)
                       .OnTrue(command_Outtake(&m_Intake, &m_LED).ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}
