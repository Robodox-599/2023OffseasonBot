// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include "Constants.h"

#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>

#include "frc/XboxController.h"
#include "frc2/command/button/JoystickButton.h"
#include <frc/Joystick.h>
#include <frc2/command/button/CommandXboxController.h>

#include "subsystems/subsystem_Intake.h"
#include "commands/command_Intake.h"
#include "commands/command_Outtake.h"

#include "subsystems/subsystem_LED.h"
#include "commands/command_SetLED.h"
#include "commands/command_Blink.h"

#include "subsystems/ExampleSubsystem.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc2::CommandXboxController m_driverController{
      OperatorConstants::kDriverControllerPort};

  // The robot's subsystems are defined here...
  ExampleSubsystem m_subsystem;
  subsystem_Intake m_Intake;
  subsystem_LED m_LED;

  frc::XboxController XboxDrive{ControllerConstants::XboxDriveID};
  frc::XboxController XboxYaperator{ControllerConstants::XboxYaperatorID};

  //frc2::InstantCommand m_cycleLEDs{[this] { m_LED.cycleLED(); }, {&m_LED}};

  void ConfigureBindings();
  
};
