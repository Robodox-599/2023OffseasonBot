// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.ControllerConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final Joystick driveRad = new Joystick(ControllerConstants.driveRadID);
  private final Joystick haPerator = new Joystick(ControllerConstants.xboxHaperatorID);

  /* Drive Controls */
  private final int translationAxis = ControllerConstants.xboxLYAxis;
  private final int strafeAxis = ControllerConstants.xboxLXAxis;
  private final int rotationAxis = ControllerConstants.xboxRXAxis;

  private final int zeroGyroButton = ControllerConstants.xboxY;
  private final int robotCentricButton = ControllerConstants.xboxLB;
  private final int zeroAngleButton = ControllerConstants.xboxA;

  /* Driver Buttons */
  private final JoystickButton zeroGyro =
      new JoystickButton(driveRad, zeroGyroButton);
  private final JoystickButton robotCentric =
      new JoystickButton(driveRad, robotCentricButton);
  private final JoystickButton zeroAngle = 
      new JoystickButton(driveRad, zeroAngleButton);

  /* Operator Controls */
  private final int wristAxis = ControllerConstants.xboxLYAxis;
  private final int elevatorAxis = ControllerConstants.xboxRYAxis;

  private final int highConeButton = ControllerConstants.xboxY;
  private final int midConeButton = ControllerConstants.xboxB;
  private final int highCubeButton = ControllerConstants.xboxX;
  private final int midCubeButton = ControllerConstants.xboxA;

  private final int toggleLEDButton = ControllerConstants.xboxLeftJoyPress;
  private final int doubleSubButton = ControllerConstants.xboxLB;
  private final int groundIntakeButton = ControllerConstants.xboxView; // a.k.a Back Button
  private final int stowButton = ControllerConstants.xboxMenu; // a.k.a Start Button
  
  /* Operator Buttons */
  private final JoystickButton highCone =
      new JoystickButton(haPerator, highConeButton
      );
  private final JoystickButton midCone =
      new JoystickButton(haPerator, midConeButton
      );
  private final JoystickButton highCube =
      new JoystickButton(haPerator, highCubeButton
      );
  private final JoystickButton midCube =
      new JoystickButton(haPerator, midCubeButton
      );
  private final JoystickButton toggleLED =
      new JoystickButton(haPerator, toggleLEDButton
      );
  private final JoystickButton doubleSub =
      new JoystickButton(haPerator, doubleSubButton
      );
  private final JoystickButton groundIntake =
      new JoystickButton(haPerator, groundIntakeButton
      );
  private final JoystickButton stow =
      new JoystickButton(haPerator, stowButton
      );

  /* Subsystems */
  private final subsystem_DriveTrain s_Swerve = new subsystem_DriveTrain();
  private final subsystem_Elevator s_Elevator = new subsystem_Elevator();
  private final subsystem_Wrist s_Wrist = new subsystem_Wrist();
  private final subsystem_Intake s_Intake = new subsystem_Intake();
  private final subsystem_LEDs s_LED = new subsystem_LEDs();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_Swerve.setDefaultCommand(
        new command_DriveTeleop(
            s_Swerve,
            () -> {return -driveRad.getRawAxis(translationAxis);},
            () -> {return -driveRad.getRawAxis(strafeAxis);},
            () -> {return -driveRad.getRawAxis(rotationAxis);},
            () -> {return robotCentric.getAsBoolean();}));

    // s_Elevator.setDefaultCommand(
    //     new command_MoveElevatorManually(s_Elevator, 
    //     () -> {return -haPerator.getRawAxis(elevatorAxis);})
    // );

    // s_Wrist.setDefaultCommand(
    //   new command_MoveWristManually(s_Wrist, 
    //   () -> {return -haPerator.getRawAxis(wristAxis);})
    // );
    
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    // zeroGyro.whenPressed(new InstantCommand(() -> s_Swerve.zeroGyro()));
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

    highCone.onTrue(cGroup_Elevator.scoreHighCone(s_Elevator, s_Wrist, s_Intake, s_LED));

    midCone.onTrue(cGroup_Elevator.scoreMidCone(s_Elevator, s_Wrist, s_Intake, s_LED));

    highCube.onTrue(cGroup_Elevator.scoreHighCube(s_Elevator, s_Wrist, s_Intake, s_LED));
    
    midCube.onTrue(cGroup_Elevator.scoreMidCube(s_Elevator, s_Wrist, s_Intake, s_LED));
    
    toggleLED.onTrue(s_LED.cycleLEDCommand());

    doubleSub.onTrue(cGroup_Elevator.doubleSubstationIntake(s_Elevator, s_Wrist, s_Intake, s_LED));

    groundIntake.onTrue(cGroup_Elevator.groundIntake(s_Elevator, s_Wrist, s_Intake, s_LED));

    stow.onTrue(cGroup_Elevator.toStow(s_Elevator, s_Wrist));

    zeroAngle.onTrue(s_Swerve.zeroModuleAngles());
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new Autos(s_Swerve);
  }
}
