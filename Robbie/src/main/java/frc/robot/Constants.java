// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.RotatedRect;

import edu.wpi.first.math.geometry.Rotation2d;

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

  public static class ArmConstants{
    public static final int DEVICE_ID_ARM_MASTER = 10;
    public static final int DEVICE_ID_ARM_SLAVE = 11;

    public static final double ARM_GEAR_RATIO = 25;

    public static final Rotation2d TOLERANCE = Rotation2d.fromRotations(2);

    public static final double MAX_ACCELERATION = 5000;
    public static final double MAX_VELOCITY = 5000;
    public static final double ARM_kP = 0.02;
    public static final double ARM_kD = 0.00021;
  }

  public static class ExtenderConstants{
    public static final int DEVICE_ID_EXTENDER = 12;
  
    public static final double MAX_ACCELERATION = 2000;
    public static final double MAX_VELOCITY = 2000;

    public static final double TOLERANCE = 5;

    public static final double EXTENDER_kP = 0.02;
    public static final double EXTENDER_kD = 0.0002;
  
  }

  public static class ClawConstants{
    public static final int DEVICE_ID_FORWARD = 0;
    public static final int DEVICE_ID_REVERSE = 1;

    public static final int DEVICE_ID_PH = 13;
  }

  public static class SuperStructureConstants{

  }
}
