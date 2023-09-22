// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.nav.EFPathingTelemetrySub;

import java.awt.geom.Point2D;

public class GrabberSubsystem extends SubsystemBase {
  /** Creates a new GrabberSubsystem. */
  private StageOneSubsystem stageOneSub;
  private StageTwoSubsystem stageTwoSub;

  double stageOneSetpoint = Units.degreesToRadians(70);
  double stageTwoSetpoint = Units.degreesToRadians(-140);
  EFPathingTelemetrySub efPathingTelemetrySub;

  public GrabberSubsystem(EFPathingTelemetrySub efPathingTelemetrySub) {
    this.efPathingTelemetrySub = efPathingTelemetrySub;
    stageOneSub = new StageOneSubsystem();
    stageTwoSub = new StageTwoSubsystem();


  }

  public double[] thetaPhiToXYMeters(double stageOneAng, double stageTwoAng){
    double x = 0,y = 0;
    x += Units.inchesToMeters(Constants.ArmConstants.stageOneLength) * Math.cos(stageOneAng);
    y += Units.inchesToMeters(Constants.ArmConstants.stageOneLength) * Math.sin(stageOneAng);

    x += Units.inchesToMeters(Constants.ArmConstants.stageTwoLength) * Math.cos(stageOneAng + stageTwoAng);
    y += Units.inchesToMeters(Constants.ArmConstants.stageTwoLength) * Math.sin(stageOneAng + stageTwoAng);
    return new double[]{x,y};
  }
double lastControllerUpdateTime = Timer.getFPGATimestamp();

  public void joystickDrive(double stageOneSpeed, double stageTwoSpeed, boolean disabled){
    if(disabled){
      stageOneSpeed = 0;
      stageTwoSpeed = 0;
    }
    double timeSinceLastUpdate = Timer.getFPGATimestamp() - lastControllerUpdateTime;
    if(Math.abs(stageOneSpeed)<0.05) stageOneSpeed = 0;
    if(Math.abs(stageTwoSpeed)<0.05) stageTwoSpeed = 0;

    stageOneSetpoint += stageOneSpeed * Units.degreesToRadians(80) * timeSinceLastUpdate;
    stageTwoSetpoint += stageTwoSpeed * Units.degreesToRadians(80) * timeSinceLastUpdate;

    stageOneSub.setAff(stageOneSpeed * 0.75);
    stageTwoSub.setAff(stageTwoSpeed * 0.25);

    lastControllerUpdateTime = Timer.getFPGATimestamp();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(stageOneSetpoint > Units.degreesToRadians(70)) stageOneSetpoint = Units.degreesToRadians(70);
    if(stageOneSetpoint < Units.degreesToRadians(45)) stageOneSetpoint = Units.degreesToRadians(45);

    if(stageTwoSetpoint > Units.degreesToRadians(-50)) stageTwoSetpoint = Units.degreesToRadians(-50);
    if(stageTwoSetpoint < Units.degreesToRadians(-150)) stageTwoSetpoint = Units.degreesToRadians(-150);



    efPathingTelemetrySub.updateStageOneAngle(stageOneSub.getAngle());
    efPathingTelemetrySub.updateStageTwoAngle(stageTwoSub.getAngle());

    SmartDashboard.putNumber("stageOneSetpoint",Units.radiansToDegrees(stageOneSetpoint));
    SmartDashboard.putNumber("stageTwoSetpoint",Units.radiansToDegrees(stageTwoSetpoint));

    stageOneSub.setDesiredAngle(stageOneSetpoint);
    stageTwoSub.setDesiredAngle(stageTwoSetpoint);

  }

  public void goHome(){
    stageOneSetpoint = Units.degreesToRadians(70);
    stageTwoSetpoint = Units.degreesToRadians(-140);
  }
}
