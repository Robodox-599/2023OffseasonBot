// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;
import frc.robot.Constants.BackLeftModule;
import frc.robot.Constants.BackRightModule;
import frc.robot.Constants.FrontLeftModule;
import frc.robot.Constants.FrontRightModule;
import frc.robot.Constants.SwerveConstants;

public class subsystem_DriveTrain extends SubsystemBase {
  /** Creates a new subsystem_DriveTrain. */
  private SwerveModule m_FrontLeft;
  private SwerveModule m_FrontRight;
  private SwerveModule m_BackLeft;
  private SwerveModule m_BackRight;
  private Pigeon2 m_Gyro;

  private boolean m_IsMathed;

  public subsystem_DriveTrain() {
    m_FrontLeft = new SwerveModule(
                      FrontLeftModule.driveMotorID,
                      FrontLeftModule.angleMotorID,
                      FrontLeftModule.driveEncInvert,
                      FrontLeftModule.angleEncInvert);
    
    m_FrontRight = new SwerveModule(
                      FrontRightModule.driveMotorID,
                      FrontRightModule.angleMotorID,
                      FrontRightModule.driveEncInvert,
                      FrontRightModule.angleEncInvert);
    
    m_BackLeft = new SwerveModule(
                      BackLeftModule.driveMotorID,
                      BackLeftModule.angleMotorID,
                      BackLeftModule.driveEncInvert,
                      BackLeftModule.angleEncInvert);
    
    m_BackRight = new SwerveModule(
                      BackRightModule.driveMotorID,
                      BackRightModule.angleMotorID,
                      BackRightModule.driveEncInvert,
                      BackRightModule.angleEncInvert);
    
    m_Gyro = new Pigeon2(12, "BarryDriveCANivore");
    Timer.delay(0.5);
    m_Gyro.configFactoryDefault();
    Timer.delay(0.5);
    m_Gyro.setYaw(0.0);

    m_IsMathed = false;
  }

  public void drive(Translation2d translation, double zRot){
    if(m_IsMathed){ 
      SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    
      /* Angular Velocity */
      double omega_x = zRot * SwerveConstants.wheelBase / 2.0;
      double omega_y = zRot * SwerveConstants.trackWidth / 2.0;
      
      /* Linear Velocity */
      double vel_x = translation.getX();
      double vel_y = translation.getY();
 
      /* Arbitrary Variable Definitions */
      double arb_A = vel_x - omega_x;
      double arb_B = vel_x + omega_x;
      double arb_C = vel_y - omega_y;
      double arb_D = vel_y + omega_y;

      /* Front Left Module */
      double vel_FL = Math.hypot(arb_B, arb_D);
      double theta_FL = Math.atan2(arb_B, arb_D) * 180.0 / Math.PI;
      SmartDashboard.putNumber("FL Mathed Angle", theta_FL);
      moduleStates[0] = new SwerveModuleState(vel_FL, Rotation2d.fromDegrees(theta_FL));

      /* Front Right Module */
      double vel_FR = Math.hypot(arb_B, arb_C);
      double theta_FR = Math.atan2(arb_B, arb_C) * 180.0 / Math.PI;
      SmartDashboard.putNumber("FR Mathed Angle", theta_FR);
      moduleStates[1] = new SwerveModuleState(vel_FR, Rotation2d.fromDegrees(theta_FR));

      /* Back Left Module */
      double vel_BL = Math.hypot(arb_A, arb_D);
      double theta_BL = Math.atan2(arb_A, arb_D) * 180.0 / Math.PI;
      SmartDashboard.putNumber("BL Mathed Angle", theta_BL);
      moduleStates[2] = new SwerveModuleState(vel_BL, Rotation2d.fromDegrees(theta_BL));

      /* Back Right Module */
      double vel_BR = Math.hypot(arb_A, arb_C);
      double theta_BR = Math.atan2(arb_A, arb_C) * 180.0 / Math.PI;
      SmartDashboard.putNumber("BR Mathed Angle", theta_BR);
      moduleStates[3] = new SwerveModuleState(vel_BR, Rotation2d.fromDegrees(theta_BR));

      SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.maxSpeed);
      setModuleStates(moduleStates);

    } else {
      SwerveModuleState[] moduleStates = 
          SwerveConstants.swerveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
              translation.getX(), translation.getY(), zRot, Rotation2d.fromDegrees(m_Gyro.getYaw())));
      SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.maxSpeed);
      setModuleStates(moduleStates);
    }
  }
  
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxSpeed);
    m_FrontLeft.setDesiredState(desiredStates[0]);
    m_FrontRight.setDesiredState(desiredStates[1]);
    m_BackLeft.setDesiredState(desiredStates[2]);
    m_BackRight.setDesiredState(desiredStates[3]);
  }
  
  public void stopModules() {
    m_FrontLeft.stop();
    m_FrontRight.stop();
    m_BackLeft.stop();
    m_BackRight.stop();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("FL Current Angle", m_FrontLeft.getAngle());
    SmartDashboard.putNumber("FR Current Angle", m_FrontRight.getAngle());
    SmartDashboard.putNumber("BL Current Angle", m_BackLeft.getAngle());
    SmartDashboard.putNumber("BR Current Angle", m_BackRight.getAngle());
    SmartDashboard.putNumber("Pose Yaw", m_Gyro.getYaw());
  }
}