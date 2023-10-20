// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class IntakeConstants{
    public static enum intakeState{
    noGamepiece,
    hasCone,
    hasCube,
    coneIntaking, 
    coneOuttaking, 
    cubeIntaking, 
    cubeOuttaking
  };
  public static final double normalOutputCurrent = 30.0;
  public static final int IntakeMotorID = 3;
  public static final int IntakeMotorCurrentLimit = 50;
  public static final double CurrentSpikeTimeSeconds = 0.15;
}

  public static class LEDConstants{
    public static enum LEDState{
      Standby, Yellow, Purple, Error, Off
  };    
  public static final int CANdleID = 20;
  public static final int IntakeFlashTimeSeconds = 1;
  }

  public static class ControllerConstants{
    public static final double Deadband = 0.1;
    public static final double TriggerActivate = 0.8;
    
    public static final int XboxDriveID = 0;
    public static final int XboxOperatorID = 1;

    public static final int xboxLXAxis = 0;
    public static final int xboxLYAxis = 1;
    public static final int xboxRXAxis = 4;
    public static final int xboxRYAxis = 5;

    public static final int xboxLTAxis = 2;
    public static final int xboxRTAxis = 3;

    public static final int xboxA = 1;
    public static final int xboxB = 2;
    public static final int xboxX = 3;
    public static final int xboxY = 4;
    public static final int xboxLB = 5;
    public static final int xboxRB = 6;
    public static final int xboxView = 7;
    public static final int xboxMenu = 8;
    public static final int xboxLeftJoyPress = 9;
    public static final int xboxRightJoyPress = 10;
    public static final int xboxRightDPad = 11;
}

}
