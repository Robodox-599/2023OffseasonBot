// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
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
  private WPI_Pigeon2 m_Gyro;

  private SwerveDriveKinematics m_Kinematics;
  private SwerveModulePosition[] m_ModulePositions;
  private SwerveDrivePoseEstimator m_PoseEstimator;

  private Vector<N3> vec1 = VecBuilder.fill(0.7, 0.7, 0.1);
  private Vector<N3> vec2 = VecBuilder.fill(0.3, 0.3, 0.9);

  public subsystem_DriveTrain() {
    m_FrontLeft = new SwerveModule(
                      FrontLeftModule.driveMotorID,
                      FrontLeftModule.angleMotorID,
                      FrontLeftModule.driveMotInvert,
                      FrontLeftModule.angleMotInvert);
    
    m_FrontRight = new SwerveModule(
                      FrontRightModule.driveMotorID,
                      FrontRightModule.angleMotorID,
                      FrontRightModule.driveMotInvert,
                      FrontRightModule.angleMotInvert);
    
    m_BackLeft = new SwerveModule(
                      BackLeftModule.driveMotorID,
                      BackLeftModule.angleMotorID,
                      BackLeftModule.driveMotInvert,
                      BackLeftModule.angleMotInvert);
    
    m_BackRight = new SwerveModule(
                      BackRightModule.driveMotorID,
                      BackRightModule.angleMotorID,
                      BackRightModule.driveMotInvert,
                      BackRightModule.angleMotInvert);
    
    m_Gyro = new WPI_Pigeon2(12, "rio");
    Timer.delay(0.5);
    m_Gyro.configFactoryDefault();
    Timer.delay(0.5);
    m_Gyro.setYaw(0.0);

    m_Kinematics = new SwerveDriveKinematics(SwerveConstants.frontLeft,
                                            SwerveConstants.frontRight,
                                            SwerveConstants.backLeft,
                                            SwerveConstants.backRight);

    m_ModulePositions = new SwerveModulePosition[4];
    m_ModulePositions[0] = m_FrontLeft.getPosition();
    m_ModulePositions[1] = m_FrontRight.getPosition();
    m_ModulePositions[2] = m_BackLeft.getPosition();
    m_ModulePositions[3] = m_BackRight.getPosition();

    m_PoseEstimator = new SwerveDrivePoseEstimator(m_Kinematics,
                                                  new Rotation2d(),
                                                  m_ModulePositions,
                                                  new Pose2d(0.0, 0.0, 
                                                  new Rotation2d()), 
                                                  vec1, vec2);
  }

  public void drive(double xSpeed, double ySpeed, double zRot){
    SwerveModuleState[] moduleStates = 
          SwerveConstants.swerveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
              xSpeed, ySpeed, zRot, Rotation2d.fromDegrees(m_Gyro.getYaw())));
      SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.maxSpeed);
      setModuleStates(moduleStates);

    // SmartDashboard.putNumber("FL Desired Speed", moduleStates[0].speedMetersPerSecond);
    SmartDashboard.putNumber("FL Desired Angle", moduleStates[0].angle.getDegrees());
    // SmartDashboard.putNumber("FR Desired Speed", moduleStates[1].speedMetersPerSecond);
    SmartDashboard.putNumber("FR Desired Angle", moduleStates[1].angle.getDegrees());
    // SmartDashboard.putNumber("BL Desired Speed", moduleStates[2].speedMetersPerSecond);
    SmartDashboard.putNumber("BL Desired Angle", moduleStates[2].angle.getDegrees());
    // SmartDashboard.putNumber("BR Desired Speed", moduleStates[3].speedMetersPerSecond);
    SmartDashboard.putNumber("BR Desired Angle", moduleStates[3].angle.getDegrees());
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
    SmartDashboard.putNumber("Gyro Yaw", m_Gyro.getYaw());
    SmartDashboard.putNumber("Pose Yaw", m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees());
    m_PoseEstimator.update(m_Gyro.getRotation2d(), m_ModulePositions);
    
  }
}