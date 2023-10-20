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

  public class OperatorConstants {
    public static final int kDriverControllerPort = 1;
    public static final int xboxLXAxis = 0;
    public static final int xboxLYAxis = 1;
    public static final int xboxRXAxis = 4;
    public static final int xboxRYAxis = 5;
    public static final double ManualThreshold = 0.02;
    public static final int xboxLTAxis = 2;
    public static final int xboxRTAxis = 3;
}

public class WristConstants {
    public static final double kWristI = 0.0;
    public static final double kWristP = 0.05;
    public static final double kWristD = 0.0005;
    public static final double start_pos = -0.5; // hard stop pos
    public static final double bufferZone = 0.0; // still needs defining
    public static final double stepperConstant = 0.1;
    public static final double TargetTime = 0.1; // assuming this is a double value
    public static final int motorID = 12;
    public static final double WristThreshold = -14.0;
}

public class WristPositions {
    public static final double HighConePos = 14.91; // 125.81
    // needs defining
    public static final double HighCubePos = 7.96; // 67.16
    public static final double MidConePos = 9.96; // 84.03
    public static final double MidCubePos = 9.96;
    public static final double SubstationPos = 10.9048; // fix
    public static final double GroundIntake = 16.809; // fix
    // public static final double WristZero = -0.5; // fix !!! this is start_pos
}

}
