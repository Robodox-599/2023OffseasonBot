// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems heresdfsdfsdfsdfsdfsfs3ruy37v9ny32

  // Configure the button bindings

  m_ElevatorMotor.SetDefaultCommand(command_MoveElevatorManually(&m_ElevatorMotor,
                        [this]{return -XboxDrive.GetRawAxis(ControllerConstants::xboxRXAxis);}).ToPtr());

  //[this]{return -XboxDrive.GetRawAxis(ControllerConstants::xboxRXAxis);}).ToPtr());

  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  frc2::Trigger([this] {
    return m_subsystem.ExampleCondition();
  }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  // m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());

  //Y is high cone
  //B is mid cone
  //X is high cube
  //A is mid cube
  //Left bumper is zero
  //Manual is axis of smth smth

frc2::JoystickButton(&XboxDrive, 
                      frc::XboxController::Button::kY)
                      .OnTrue(command_MoveElevator(&m_ElevatorMotor, 
                                                  [=] {return ElevatorConstants::kElevatorScoreHighCone;}, 
                                                  [=] {return false;}, 
                                                  [=] {return ElevatorConstants::ElevatorThreshold;}).ToPtr());


frc2::JoystickButton(&XboxDrive, 
                      frc::XboxController::Button::kB)
                      .OnTrue(command_MoveElevator(&m_ElevatorMotor, 
                                                  [=] {return ElevatorConstants::kElevatorScoreMidCone;}, 
                                                  [=] {return false;}, 
                                                  [=] {return ElevatorConstants::ElevatorThreshold;}).ToPtr());


frc2::JoystickButton(&XboxDrive, 
                      frc::XboxController::Button::kX)
                      .OnTrue(command_MoveElevator(&m_ElevatorMotor, 
                                                  [=] {return ElevatorConstants::kElevatorScoreHighCube;}, 
                                                  [=] {return false;}, 
                                                  [=] {return ElevatorConstants::ElevatorThreshold;}).ToPtr());


frc2::JoystickButton(&XboxDrive, 
                      frc::XboxController::Button::kA)
                      .OnTrue(command_MoveElevator(&m_ElevatorMotor, 
                                                  [=] {return ElevatorConstants::kElevatorScoreMidCube;}, 
                                                  [=] {return false;}, 
                                                  [=] {return ElevatorConstants::ElevatorThreshold;}).ToPtr());

frc2::JoystickButton(&XboxDrive, 
                      frc::XboxController::Button::kLeftBumper)
                      .OnTrue(command_MoveElevator(&m_ElevatorMotor, 
                                                  [=] {return ElevatorConstants::kElevatorStow;}, 
                                                  [=] {return false;}, 
                                                  [=] {return ElevatorConstants::ElevatorThreshold;}).ToPtr());


}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}







                     