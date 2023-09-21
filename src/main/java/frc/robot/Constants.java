// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;

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

  public static final class OIConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    public static final int DRIVER_Y_AXIS = XboxController.Axis.kLeftY.value;
    public static final int DRIVER_X_AXIS = XboxController.Axis.kLeftX.value;
    public static final int DRIVER_ROT_AXIS = XboxController.Axis.kRightX.value;
    public static final int DRIVER_FIELD_ORIENTED_BUTTON_IDX = XboxController.Button.kRightBumper.value;

    public static final double DEADBAND = 0.06; //0.0275-0.03 //0.06
  }

  public static final class ModuleConstants {
    //Physical
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.8);
    public static final double kDriveMotorGearRatio = 6.75;
    public static final double kTurningMotorGearRatio = 21.4286;

    //Conversions
    public static final double FALCON_TICKS = 2048;
    public static final double RADIANS_TO_FALCON = FALCON_TICKS / (2 * Math.PI);
    public static final double RADIANS_TO_TURNING = RADIANS_TO_FALCON * kTurningMotorGearRatio;
    public static final double WHEEL_CIRCUMFERENCE = kWheelDiameterMeters * Math.PI;
    public static final double METERS_TO_ROTATIONS = (1 / WHEEL_CIRCUMFERENCE);
    public static final double METERS_TO_DRIVE = METERS_TO_ROTATIONS * kDriveMotorGearRatio * FALCON_TICKS;
    public static final double METERS_TO_DRIVE_VELOCITY = METERS_TO_DRIVE / 10;
    public static final double ABS_TO_RADIANS = 2.0 * Math.PI;

    //Gains
    //Turn
    public static final double kPTurning = 0.21; //0.21 //works from 0.1-0.3 but 0.21 seems to offer low chattering and pretty quick alignment
    //Drive
    public static final double kAFFDrive = 0.015; //0.0151 //0.015
    public static final double kFDrive = .045012; //0.04390375 //0.03751 //.045012
    public static final double kPDrive = 0.02; //0.08 //0.02

    public static final double NEUTRAL_DEADBAND = 0.01;
  }

  public static class ArmConstants {
    public static final int stageOneLeftId =  11;
    public static final int stageOneRightId = 13;

    public static final double continuousCurrentLimit = 20;
    public static final double peakCurrentLimit = 40;
    public static final double peakCurrentTime = 250;

    public static final int stageTwoLeftId = 49;
    public static final int stageTwoRightId = 48;

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

  public static final class DriveConstants {

    public static final double TRACK_WIDTH = Units.inchesToMeters(23.625); //need to find
    // Distance between right and left wheels
    public static final double WHEEL_BASE = Units.inchesToMeters(23.625); //need to find


    // Distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
    );

    //TBD
    public static final int FRONT_LEFT_DRIVE_MOTOR_PORT = 1;
    public static final int FRONT_RIGHT_DRIVE_MOTOR_PORT = 6;
    public static final int BACK_LEFT_DRIVE_MOTOR_PORT = 8;
    public static final int BACK_RIGHT_DRIVE_MOTOR_PORT = 3;

    //TBD
    public static final int FRONT_LEFT_TURNING_MOTOR_PORT = 4;
    public static final int FRONT_RIGHT_TURNING_MOTOR_PORT = 5;
    public static final int BACK_LEFT_TURNING_MOTOR_PORT = 2;
    public static final int BACK_RIGHT_TURNING_MOTOR_PORT = 7;

    public static final boolean FRONT_LEFT_DRIVE_ENCODER_REVERSED = false;
    public static final boolean FRONT_RIGHT_DRIVE_ENCODER_REVERSED = true;
    public static final boolean BACK_LEFT_DRIVE_ENCODER_REVERSED = false;
    public static final boolean BACK_RIGHT_DRIVE_ENCODER_REVERSED = true;

    public static final boolean FRONT_LEFT_TURNING_ENCODER_REVERSED = true;
    public static final boolean FRONT_RIGHT_TURNING_ENCODER_REVERSED = true;
    public static final boolean BACK_LEFT_TURNING_ENCODER_REVERSED = true;
    public static final boolean BACK_RIGHT_TURNING_ENCODER_REVERSED = true;

    //TBD
    public static final int FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_PORT = 0;
    public static final int FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_PORT = 1;
    public static final int BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_PORT = 2;
    public static final int BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_PORT = 3;

    //sort of calculated
    public static final double FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD = -2.260 -0.140 +Math.PI; //-2.29+Math.PI
    public static final double FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD = -5.30+Math.PI; //-6.06+Math.PI
    public static final double BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD = -5.420+Math.PI; //-2.25
    public static final double BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD = -3.925+Math.PI; //-0.75

    public static final boolean FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;
    public static final boolean BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_REVERSED = false;

    public static final double RADIUS = Units.inchesToMeters(32/2);

    public static final double PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 4.8; //13.3 adjusted, 16.4 free //empirical 4.8 meters when not on ground
    public static final double PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = PHYSICAL_MAX_SPEED_METERS_PER_SECOND / RADIUS;

    public static final double TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND = Units.feetToMeters(100); //10 //100 for testing
    public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND * 2 * Math.PI; //idk
    public static final double TELE_DRIVE_MAX_SPEED_METERS_PER_SECOND = PHYSICAL_MAX_SPEED_METERS_PER_SECOND;
    public static final double TELE_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = PHYSICAL_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

    public static final double DEGREES_TO_RADIANS = (2*Math.PI) / 360;

    public static final double kPTurning = 0.0015; //0.0015 low-no oscillation
    public static final double kDTurning = 0.0; //0.0 unnecissary

    public static final double kPFudge = 0.02; //0.2 seems pretty close

    public static final boolean USE_NAV_X_OVER_PIGEON = false;


    public static final double posTolerance = Units.inchesToMeters(0.25);
    public static final double rotationTolerance = 0.2; //adds half an inch with arm fully extended


    public static final double GYRO_Z_ERROR = 0.674; //.674
    public static final double GYRO_MOUNT_POSE_PITCH = 0;
    public static final double GYRO_MOUNT_POSE_YAW = 0;
    public static final double GYRO_MOUNT_POSE_ROLL = 0;

    public static double drivePValue = 3; //% speed for every meter away from target
    public static  double turnPValue = 0.010; //% speed for every degree away from target

    public static final double maxPowerOut = 0.3;
    public static final double maxTurningPowerOut = 0.2;

    public static final double maxPowerOutForAssist = 0.1;
    public static final double maxTurningPowerOutForAssist = 0.1;

  }


}
