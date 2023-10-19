// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"
#include "commands/command_RunMotor.h"
#include "commands/command_ManualMotor.h"


RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  // m_motor.SetDefaultCommand(command_RunMotor(&m_motor, [this]{return m_drivercontroller;})); //FIX THIS WITH XBOX mappings
  // m_motor.SetDefaultCommand(command_RunMotor(&m_motor, [=] {return WristPositions::HighConePos;}, [=] {return false;}, 
  //                                           [=] {return WristConstants::WristThreshold;}).ToPtr());
  // // Configure the button bindings
  // m_motor.ResetWrist();
  m_motor.SetDefaultCommand(command_ManualMotor(&m_motor,
                          [this]{return -m_driverController.GetRawAxis(OperatorConstants::xboxLYAxis);}//replace "this" with = if problem
                       ).ToPtr());
  // frc::XboxController::Axis::
  ConfigureBindings();
}
 void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here
  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
   frc2::JoystickButton(&m_driverController, 
                       frc::XboxController::Button::kY)
                       .OnTrue(command_RunMotor(&m_motor, 
                                            [=] {return WristPositions::HighConePos;}, 
                                            [=] {return false;}, 
                                            [=] {return WristConstants::WristThreshold;},
                                            [=] {return 0;}
                       ).ToPtr());
  
  frc2::JoystickButton(&m_driverController, 
                       frc::XboxController::Button::kX)
                       .OnTrue(command_RunMotor(&m_motor, 
                                            [=] {return WristPositions::HighCubePos;}, 
                                            [=] {return false;}, 
                                            [=] {return WristConstants::WristThreshold;},
                                            [=] {return 0;}
                       ).ToPtr());

  frc2::JoystickButton(&m_driverController, 
                       frc::XboxController::Button::kB)
                       .OnTrue(command_RunMotor(&m_motor, 
                                            [=] {return WristPositions::MidConePos;}, 
                                            [=] {return false;}, 
                                            [=] {return WristConstants::WristThreshold;},
                                            [=] {return 0;}
                       ).ToPtr());

  frc2::JoystickButton(&m_driverController, 
                       frc::XboxController::Button::kA)
                       .OnTrue(command_RunMotor(&m_motor, 
                                            [=] {return WristPositions::MidCubePos;}, 
                                            [=] {return false;}, 
                                            [=] {return WristConstants::WristThreshold;},
                                            [=] {return 0;}
                       ).ToPtr());
                      //add another button mapping for substation
  frc2::JoystickButton(&m_driverController,
                        frc::XboxController::Button::kLeftBumper)
                        .OnTrue(command_RunMotor(&m_motor, 
                                            [=] {return WristPositions::SubstationPos;}, 
                                            [=] {return false;}, 
                                            [=] {return WristConstants::WristThreshold;},
                                            [=] {return 0;}
                       ).ToPtr());

  frc2::JoystickButton(&m_driverController,
                        frc::XboxController::Button::kRightBumper)
                        .OnTrue(command_RunMotor(&m_motor, 
                                            [=] {return 0;}, //stow pos
                                            [=] {return false;}, 
                                            [=] {return WristConstants::WristThreshold;},
                                            [=] {return 0;}
                       ).ToPtr());
    frc2::JoystickButton(&m_driverController,
                        frc::XboxController::Button::kLeftStick)
                        .OnTrue(command_RunMotor(&m_motor, 
                                            [=] {return WristPositions::GroundIntake;}, 
                                            [=] {return false;}, 
                                            [=] {return WristConstants::WristThreshold;},
                                            [=] {return 0;}
                       ).ToPtr());
  
  // frc2::JoystickButton(&m_driverController, use this for reset writst 
  //                       frc::XboxController::Button::kRightBumper)
  //                       .OnTrue(command_RunMotor(&m_motor, 
  //                                           [=] {return WristPositions::HighConePos;}, 
  //                                           [=] {return true;}, 
  //                                           [=] {return WristConstants::wristThreshold;}
  //                      ).ToPtr());
  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  // m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}



//control guide Y is high cone, X is high cube, B is mid cone, A is mid cube, left bumper is double sub,
//right bumper is stow
//left stick is ground intake