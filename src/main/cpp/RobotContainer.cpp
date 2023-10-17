// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  m_Drive.SetDefaultCommand( command_DriveTeleop(&m_Drive, 
                                                       [this]{return frc::ApplyDeadband(-XboxDrive.GetRawAxis(ControllerConstants::xboxLYAxis), 0.05, 1.0);},
                                                       [this]{return frc::ApplyDeadband(-XboxDrive.GetRawAxis(ControllerConstants::xboxLXAxis), 0.05, 1.0);},
                                                       [this]{return frc::ApplyDeadband(-XboxDrive.GetRawAxis(ControllerConstants::xboxRXAxis), 0.05, 1.0);},
                                                       [this]{return XboxDrive.GetLeftBumperPressed();},
                                                       [this]{return XboxDrive.GetRightBumperPressed();},
                                                       [this]{return SwerveConstants::IsFieldRelative;},
                                                       [this]{return SwerveConstants::IsOpenLoop;}));
  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  // frc2::Trigger([this] {
  //   return m_subsystem.ExampleCondition();
  // }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  // // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // // pressed, cancelling on release.
  // m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());
  frc2::JoystickButton(&XboxDrive,
                       frc::XboxController::Button::kRightStick)
      .OnTrue(m_Drive.ToggleThrottleCommand());

  frc2::JoystickButton(&XboxDrive,
                        frc::XboxController::Button::kY)
      .OnTrue(m_Drive.ZeroGyroCommand());
  
  frc2::JoystickButton(&XboxDrive,
                        frc::XboxController::Button::kA)
      .OnTrue(command_Park(&m_Drive).ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}