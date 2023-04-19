// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

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
  public static class ArmConstants {
    public static final int stageOneLeftId =  11;
    public static final int stageOneRightId = 12;

    public static final double continuousCurrentLimit = 20;
    public static final double peakCurrentLimit = 40;
    public static final double peakCurrentTime = 250;

    public static final int stageTwoLeftId = 1;
    public static final int stageTwoRightId = 2;

    public static final int stageTwoSmartCurrentLimit = 40;
    public static final double stageTwoSecondaryCurrentLimit = 60;

    public static final double magEncoderCountsPerRotation = 4096;//4096
    public static final double radiansPerRotation = 2 * Math.PI;

    public static final double stageOneEncoderTicksToRadians =  (radiansPerRotation/magEncoderCountsPerRotation);

    public static final double stageOneEncoderOffset = Units.degreesToRadians(291.9 + 90 - .145);

    public static final double stageOneLength = 28.75;
    public static final double[] stageOnePivotCoordinate = {-4.864, 18.66};

    public static final double stageTwoLength = 28.75;
    public static final double stageTwoEncoderOffset = Units.degreesToRadians(43.6);//180 - 43.6 //43.6 + 180
    public static final double stageTwoEncoderRatio = 1;//32.0/22
  }

  
  public static final class EFPathingConstants{
    public static final double BUMPER_Y_FROM_ORIGIN = Units.inchesToMeters(7); //6.5 real
    public static final double BUMPER_X_FROM_ORIGIN = Units.inchesToMeters(18); //17.5 real

    public static final double EF_WIDTH = Units.inchesToMeters(17);
    public static final double EF_HEIGHT = Units.inchesToMeters(12.5);

    public static final double EF_RADIUS = Units.inchesToMeters(14.32 + 0.5); //this should be fine since it doesn't rotate?

    public static final int innerLineTestCount = 5;

    public static final double CENTER_OFFSET_FROM_PIVOT_POINT_X = EF_WIDTH/2;
    public static final double CENTER_OFFSET_FROM_PIVOT_POINT_Y = EF_HEIGHT/2;

    public static final int maxRecursionDepth = 2;
    public static final double lineDistIterator = 0.2;
    public static final double maxLineDist = 1;
    public static final int moveAngles = 8;

    public static final int minPathingDelay = 10;

    public static final double reachedInBetweenPointThreshold = 0.05;
    public static final double recalcThreshold = Units.inchesToMeters(3); // max distance to travel before recalculating trajectory


  }
}
