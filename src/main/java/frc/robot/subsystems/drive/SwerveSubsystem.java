// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.lib.MB_Math;

public class SwerveSubsystem extends SubsystemBase {
  //Modules
    private final SwerveModule frontLeft = new SwerveModule(DriveConstants.FRONT_LEFT_DRIVE_MOTOR_PORT,
                                                            DriveConstants.FRONT_LEFT_TURNING_MOTOR_PORT,
                                                            DriveConstants.FRONT_LEFT_DRIVE_ENCODER_REVERSED,
                                                            DriveConstants.FRONT_LEFT_TURNING_ENCODER_REVERSED,
                                                            DriveConstants.FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_PORT,
                                                            DriveConstants.FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD,
                                                            DriveConstants.FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_REVERSED);
    private final SwerveModule frontRight = new SwerveModule(DriveConstants.FRONT_RIGHT_DRIVE_MOTOR_PORT,
                                                            DriveConstants.FRONT_RIGHT_TURNING_MOTOR_PORT,
                                                            DriveConstants.FRONT_RIGHT_DRIVE_ENCODER_REVERSED,
                                                            DriveConstants.FRONT_RIGHT_TURNING_ENCODER_REVERSED,
                                                            DriveConstants.FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_PORT,
                                                            DriveConstants.FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD,
                                                            DriveConstants.FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_REVERSED);
    private final SwerveModule backLeft = new SwerveModule(DriveConstants.BACK_LEFT_DRIVE_MOTOR_PORT,
                                                          DriveConstants.BACK_LEFT_TURNING_MOTOR_PORT,
                                                          DriveConstants.BACK_LEFT_DRIVE_ENCODER_REVERSED,
                                                          DriveConstants.BACK_LEFT_TURNING_ENCODER_REVERSED,
                                                          DriveConstants.BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_PORT,
                                                          DriveConstants.BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD,
                                                          DriveConstants.BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_REVERSED);
    private final SwerveModule backRight = new SwerveModule(DriveConstants.BACK_RIGHT_DRIVE_MOTOR_PORT,
                                                            DriveConstants.BACK_RIGHT_TURNING_MOTOR_PORT,
                                                            DriveConstants.BACK_RIGHT_DRIVE_ENCODER_REVERSED,
                                                            DriveConstants.BACK_RIGHT_TURNING_ENCODER_REVERSED,
                                                            DriveConstants.BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_PORT,
                                                            DriveConstants.BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD,
                                                            DriveConstants.BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_REVERSED);
  //Gyro
    private WPI_Pigeon2 gyro = new WPI_Pigeon2(40);


  //Odometry
    private SwerveDrivePoseEstimator odometry;
    private Pose2d initialPose;
    private ChassisSpeeds chassisSpeeds;

  //Values
    static boolean fieldOriented = true;
  private double translationMagnitude;

  private double translationMagnitudeScaled;
  private double rotationMagnitude = 0;
  private double scaledMagnitudeRotation = 0;
  Rotation2d translationDirection;
  Rotation2d rotationDirection;

  private SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND);
  private SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.TELE_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND);
  private SlewRateLimiter turningLimiter = new SlewRateLimiter(DriveConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND);
  private PIDController turningPID = new PIDController(DriveConstants.kPTurning, 0, DriveConstants.kDTurning);

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {

   // SmartDashboard.putNumber("drivePValue",0.1);
  //  SmartDashboard.putNumber("turningPValue",0.1);

        //allows gyro to calibrate for 1 sec before requesting to reset^^
    //SmartDashboard.putNumber("kPTurning", DriveConstants.kPTurning);
  }


  public void drive(double xSpeed, double ySpeed, double turningSpeed, boolean limited, boolean fieldOriented){
   // SmartDashboard.putNumber("dr_xSpeed",xSpeed);
   // SmartDashboard.putNumber("dr_ySpeed",ySpeed);
   // SmartDashboard.putNumber("dr_rSpeed",turningSpeed);

    //  Make the driving smoother, no sudden acceleration from sudden inputs
    if(limited) {
      xSpeed = xLimiter.calculate(xSpeed * DriveConstants.TELE_DRIVE_MAX_SPEED_METERS_PER_SECOND);
      ySpeed = yLimiter.calculate(ySpeed * DriveConstants.TELE_DRIVE_MAX_SPEED_METERS_PER_SECOND);
      turningSpeed = turningLimiter.calculate(turningSpeed * DriveConstants.TELE_DRIVE_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND);
    }
    // Construct desired chassis speeds (convert to appropriate reference frames)
    ChassisSpeeds chassisSpeeds;

    SmartDashboard.putBoolean("fieldOriented", fieldOriented);

    //3.5. Fudge Factor to eliminate uncommanded change in direction when translating and rotating simultaneously
    ySpeed += turningSpeed * (-xSpeed) * DriveConstants.kPFudge;
    //debug output: ySpeed += turningSpeed * (-xSpeed) * SmartDashboard.getNumber("kPFudge", DriveConstants.kPFudge);
    xSpeed += turningSpeed * ySpeed * DriveConstants.kPFudge;
    //debug output: xSpeed += turningSpeed * ySpeed * SmartDashboard.getNumber("kPFudge", DriveConstants.kPFudge);

    // 3.55. P loops to create accurate outputs
    //turning
    //Debug intput: turningPID.setP(SmartDashboard.getNumber("kPTurning", DriveConstants.kPTurning));
    turningSpeed += turningPID.calculate(getAngularVelocity(), turningSpeed);


//4 - Convert and send chasis speeds
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);

    SmartDashboard.putNumber("driveOutSpeed",Math.sqrt(Math.pow(xSpeed,2)+Math.pow(ySpeed,2)));

    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    setModuleStates(moduleStates);
  }

  public void drivebaseXPattern(){
    SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    moduleStates[0] = new SwerveModuleState(0.02, Rotation2d.fromDegrees(45));
    moduleStates[1] = new SwerveModuleState(0.02, Rotation2d.fromDegrees(135));
    moduleStates[2] = new SwerveModuleState(0.02, Rotation2d.fromDegrees(225+90));
    moduleStates[3] = new SwerveModuleState(0.02, Rotation2d.fromDegrees(315+90));
    setModuleStates(moduleStates);
  }
  public void toggleFieldOriented(){fieldOriented = !fieldOriented;}
  public boolean getFieldOriented(){return fieldOriented;}
  public void setFieldOriented(boolean set){fieldOriented = set;}
  public void resetPose(){
    odometry.resetPosition(Rotation2d.fromDegrees(getHeading()), getPositions(), new Pose2d());
  }

  public void resetPose( Pose2d newPose){
    odometry.resetPosition(Rotation2d.fromDegrees(getHeading()), getPositions(), newPose);
  }

  public void joystickDrive(double xSpeed, double ySpeed, double turningSpeed){
    //1.5 interpret joystick data
    //translation
    translationMagnitude = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2)); //x and y vectors to polar magnitude
    translationDirection = new Rotation2d(ySpeed, xSpeed); //creates Rotation2D representing the direction of the input vectors using yspeed/xspeed as the cos/sin of the direction
    //Rotation
    rotationMagnitude = Math.abs(turningSpeed); //magnitude of joystick input
    rotationDirection = new Rotation2d(turningSpeed, 0); //conveys polarity +/-
    //Debug output: SmartDashboard.putNumber("directionR", rotationDirection.getCos());

    //1.55 scale magnitudes to reflect deadzone
    //Translation
    translationMagnitudeScaled = (1/(1- Constants.OIConstants.DEADBAND))*(translationMagnitude - Constants.OIConstants.DEADBAND); //converted to be representative of deadzone using point slope form (y-y1)=(m)(x-x1) -> (scaled-minimun raw input)=(maximum input/(1-deadzone))(raw input-deadzone) -> scaled=(maximum input/(1-deadzone))(raw input-deadzone)
    //Rotation
    scaledMagnitudeRotation = (1/(1- Constants.OIConstants.DEADBAND))*(rotationMagnitude- Constants.OIConstants.DEADBAND); //same algorithm as scaled magnitude translation

    //2.0 deadzone and construct outputs
    //Translation
    if (translationMagnitude > Constants.OIConstants.DEADBAND) {
      ySpeed = translationDirection.getCos() * translationMagnitudeScaled; //original direction, scaled magnitude... this is kinda a misnomer because this x component itself is a magnitude, but it is representative of a direction of the raw input
      xSpeed = translationDirection.getSin() * translationMagnitudeScaled;
    } else {
      xSpeed = 0.0;
      ySpeed = 0.0; //zero inputs < deadzone
    }
    //Rotation
    if (rotationMagnitude > Constants.OIConstants.DEADBAND) {
      turningSpeed = rotationDirection.getCos() * scaledMagnitudeRotation; //same as above for translation
    } else {
      turningSpeed = 0.0; //zero inputs < deadzone
    }

    /*
    // 2.5 square inputs //not needed for now, only needed it when there was a bug elsewhere
    xSpeed = xSpeed * Math.abs(xSpeed);
    ySpeed = ySpeed * Math.abs(ySpeed);
    turningSpeed = turningSpeed * Math.abs(turningSpeed);
    */
    drive(xSpeed, ySpeed, turningSpeed, true, this.fieldOriented);
  }

  //Configuration


  // constructs odometry object, called in the SwerveSubsystem constructor

  // Getters
  // A number equal to x - (y Q), where Q is the quotient of x / y rounded to the nearest integer
  // (if x / y falls halfway between two integers, the even integer is returned)
  public double getHeading() {
return 0;
  }


  // module states
  // groups each individual module state into an array to be used in other functions
  public SwerveModuleState[] getStates() {
    return new SwerveModuleState[] {frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()};
  }

  // module positions
  // groups each individual module position into an array to be used in other functions
  public SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()};
  }

  // chassis speeds
  public ChassisSpeeds getChassisSpeeds() {
    chassisSpeeds = DriveConstants.kDriveKinematics.toChassisSpeeds(getStates());
    return fieldOriented ? 
    ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond, getRotation2d()) :
    chassisSpeeds;
  }

  // get rotational velocity for closed loop
  public double getAngularVelocity() {

    return gyro.getRate() * DriveConstants.DEGREES_TO_RADIANS;
  }

  //since wpilib often wants heading in format of Rotation2d
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  //Setters 
  public void fieldOriented(boolean isFieldOriented) {
    fieldOriented = isFieldOriented;
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.PHYSICAL_MAX_SPEED_METERS_PER_SECOND);
    // normalizes wheel speeds in case max speed reached^^
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
    lastSetStates = desiredStates;
  }
  SwerveModuleState[] lastSetStates;
  double simHeading = 0;

  // stops robot movement
  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
    lastSetStates = new SwerveModuleState[]{new SwerveModuleState(),new SwerveModuleState(),new SwerveModuleState(),new SwerveModuleState()};
  }
SwerveModulePosition[] simModulePositions = new SwerveModulePosition[]{new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition(), new SwerveModulePosition()};
  double lastSimUpdateTime = Timer.getFPGATimestamp();
  double lastSimUpdateLength = 0;
  public void updateOdometry() {
    //odometry.updateWithTime(Timer.getFPGATimestamp(), new Rotation2d(Math.toRadians(getHeading())), getStates()); //make rotation difference work!!!


    //debug output: SmartDashboard.putNumber("OdoH", odometry.getEstimatedPosition().getRotation().getDegrees());
    //debug output: SmartDashboard.putNumber("gyroH", getHeading());
    //debug output: SmartDashboard.putNumber("OdoY", Units.metersToFeet(odometry.getEstimatedPosition().getY()));
    //debug output: SmartDashboard.putNumber("OdoX", Units.metersToFeet(odometry.getEstimatedPosition().getX()));      
  }
    
  public void debugOutputs() {
    //debug output: SmartDashboard.putNumber("CSH", getChassisSpeeds().omegaRadiansPerSecond);
    //debug output: SmartDashboard.putNumber("CSY", getChassisSpeeds().vyMetersPerSecond);
    //debug output: SmartDashboard.putNumber("CSX", getChassisSpeeds().vxMetersPerSecond);
   // SmartDashboard.putNumber("FRAbs",frontRight.getAbsoluteEncoderRad());
   // SmartDashboard.putNumber("FLAbs",frontLeft.getAbsoluteEncoderRad());
   // SmartDashboard.putNumber("RRAbs",backRight.getAbsoluteEncoderRad());
   // SmartDashboard.putNumber("RLAbs",backLeft.getAbsoluteEncoderRad());
  }

  // Periodic
  @Override
  public void periodic() {
    //DriveConstants.drivePValue = SmartDashboard.getNumber("drivePValue",0.1);
    //DriveConstants.turnPValue = SmartDashboard.getNumber("turningPValue",0.1);

    //constantly updates the gyro angle
  //  var gyroAngle = gyro.getRotation2d();
    //update odometry
      
    updateOdometry();

   // double[] desiredSpeeds = getDesiredSpeeds(new Pose2d(0,0,Rotation2d.fromDegrees(0)),Consta);
   // SmartDashboard.putNumber("desXto00", desiredSpeeds[0]);
  //  SmartDashboard.putNumber("desYto00", desiredSpeeds[1]);
   // SmartDashboard.putNumber("desRto00", desiredSpeeds[2]);

    //SmartDashboard.putNumber("o_x",odometry.getEstimatedPosition().getX());
    //SmartDashboard.putNumber("o_y",odometry.getEstimatedPosition().getY());
    //SmartDashboard.putNumber("o_r",odometry.getEstimatedPosition().getRotation().getDegrees());

    //dashboard outputs
      debugOutputs();
    }

public Pose2d getEstimatedPosition(){
    return odometry.getEstimatedPosition();
}

    // Vision stuff


  public SwerveDrivePoseEstimator getOdometry() {
    return odometry;
  }

  public double[] getDesiredSpeeds(Pose2d pose, double posP, double rotP){
    //get desired X and Y speed to reach a given pose
    double[] out = new double[3];
    if(pose == null) return null;
   // double rotationDiff = (pose.getRotation().getDegrees()-odometry.getEstimatedPosition().getRotation().getDegrees());
    double rotationDiff = MB_Math.angleDiffDeg(odometry.getEstimatedPosition().getRotation().getDegrees(),pose.getRotation().getDegrees());
    double xDiff = (pose.getX() - odometry.getEstimatedPosition().getX());
    double yDiff = (pose.getY() - odometry.getEstimatedPosition().getY());
    double dist = Math.sqrt(Math.pow(xDiff,2) + Math.pow(yDiff,2));

    double dirToPose = Math.atan2(yDiff,xDiff);

    out[0] = dist * Math.cos(dirToPose - odometry.getEstimatedPosition().getRotation().getRadians()) * posP;
    out[1] = dist * Math.sin(dirToPose - odometry.getEstimatedPosition().getRotation().getRadians()) * posP;  //sin and cos used to have +rotationDiff for some reason
  //  SmartDashboard.putNumber("rotationDiff",rotationDiff);
    out[2] =  rotationDiff * rotP;

    if(Math.abs(rotationDiff)< DriveConstants.rotationTolerance) out[2] = 0;
    if(dist<DriveConstants.posTolerance){
      out[0] = 0;
      out[1] = 0;
    }

    return out;
  }

  public void   driveToPose(Pose2d pose, double posP, double rotP, double maxPowerOut, double maxTurningPowerOut){
    if(pose == null) return;
    double[] speeds = getDesiredSpeeds(pose, posP, rotP);

    double ang = Math.atan2(speeds[1],speeds[0]);
    double mag = Math.sqrt(Math.pow(speeds[0],2)+Math.pow(speeds[1],2));
    if(mag>maxPowerOut){
      speeds[0] = Math.cos(ang)*maxPowerOut;
      speeds[1] = Math.sin(ang)*maxPowerOut;
    }

    //speeds[0] = MathThings.absMax(speeds[0],0.2);
    //speeds[1] = MathThings.absMax(speeds[1],0.2);
    speeds[2] = MB_Math.maxValueCutoff(speeds[2],maxTurningPowerOut);

    drive(speeds[0],speeds[1],speeds[2],true,false);
  }


  public void driveToPose(Pose2d pose, double posPa, double rotP){
    driveToPose(pose,posPa,rotP, DriveConstants.maxPowerOut, DriveConstants.maxTurningPowerOut);
  }


  double getAutoBalanceOut(double setpoint, double measurement, double p, double max){
    double out = -MB_Math.maxValueCutoff((setpoint - measurement)* p, max);
    if(Math.abs(measurement)>10) return 0.30*(measurement/Math.abs(measurement));
    return out;
  }





}