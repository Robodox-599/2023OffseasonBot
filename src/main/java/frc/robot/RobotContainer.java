// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.WristPositions;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.command_ManualMotor;
import frc.robot.commands.command_RunMotor;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.subsystem_WristMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final subsystem_WristMotor m_motor = new subsystem_WristMotor();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_motor.setDefaultCommand(new command_ManualMotor(m_motor, () -> -m_driverController.getRawAxis(OperatorConstants.xboxLYAxis)));
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    new JoystickButton(m_driverController, 
                      XboxController.Button.kY.value)
                      .onTrue(new command_RunMotor(m_motor, 
                      () -> WristPositions.HighConePos,
                      () -> false,
                      () -> WristConstants.WristThreshold,
                      () -> 0
                      ));
    new JoystickButton(m_driverController, 
                      XboxController.Button.kX.value)
                      .onTrue(new command_RunMotor(m_motor, 
                      () -> WristPositions.HighCubePos,
                      () -> false,
                      () -> WristConstants.WristThreshold,
                      () -> 0
                      ));
    new JoystickButton(m_driverController, 
                      XboxController.Button.kB.value)
                      .onTrue(new command_RunMotor(m_motor, 
                      () -> WristPositions.MidConePos,
                      () -> false,
                      () -> WristConstants.WristThreshold,
                      () -> 0
                      ));
    new JoystickButton(m_driverController, 
                      XboxController.Button.kA.value)
                      .onTrue(new command_RunMotor(m_motor, 
                      () -> WristPositions.MidCubePos,
                      () -> false,
                      () -> WristConstants.WristThreshold,
                      () -> 0
                      ));
    new JoystickButton(m_driverController, 
                      XboxController.Button.kLeftBumper.value)
                      .onTrue(new command_RunMotor(m_motor, 
                      () -> WristPositions.HighConePos,
                      () -> false,
                      () -> WristConstants.WristThreshold,
                      () -> 0
                      ));
    new JoystickButton(m_driverController, 
                      XboxController.Button.kRightBumper.value)
                      .onTrue(new command_RunMotor(m_motor, 
                      () -> 0, //stow pos
                      () -> false,
                      () -> WristConstants.WristThreshold,
                      () -> 0
                      ));
    new JoystickButton(m_driverController, 
                      XboxController.Button.kLeftStick.value)
                      .onTrue(new command_RunMotor(m_motor, 
                      () -> WristPositions.GroundIntake,
                      () -> false,
                      () -> WristConstants.WristThreshold,
                      () -> 0
                      ));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
