// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.nav.EFPathingTelemetrySub;

import java.awt.geom.Point2D;

public class GrabberSubsystem extends SubsystemBase {
  /** Creates a new GrabberSubsystem. */
  private StageOneSubsystem stageOneSub;
  private StageTwoSubsystem stageTwoSub;

  double stageOneSetpoint = 0;
  double stageTwoSetpoint = 0;
  EFPathingTelemetrySub efPathingTelemetrySub;

  public GrabberSubsystem(EFPathingTelemetrySub efPathingTelemetrySub) {
    this.efPathingTelemetrySub = efPathingTelemetrySub;
    stageOneSub = new StageOneSubsystem();
    stageTwoSub = new StageTwoSubsystem();
    SmartDashboard.putNumber("stageOneSetpoint",60);
    SmartDashboard.putNumber("stageTwoSetpoint",-152);

  }

  public double[] thetaPhiToXYMeters(double stageOneAng, double stageTwoAng){
    double x = 0,y = 0;
    x += Units.inchesToMeters(Constants.ArmConstants.stageOneLength) * Math.cos(stageOneAng);
    y += Units.inchesToMeters(Constants.ArmConstants.stageOneLength) * Math.sin(stageOneAng);

    x += Units.inchesToMeters(Constants.ArmConstants.stageTwoLength) * Math.cos(stageOneAng + stageTwoAng);
    y += Units.inchesToMeters(Constants.ArmConstants.stageTwoLength) * Math.sin(stageOneAng + stageTwoAng);
    return new double[]{x,y};
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    efPathingTelemetrySub.updateStageOneAngle(stageOneSub.getAngle());
    efPathingTelemetrySub.updateStageTwoAngle(stageTwoSub.getAngle());
    efPathingTelemetrySub.updatePivotPoint(new Point2D.Double(2,2));


    stageTwoSub.setDesiredAngle(Units.degreesToRadians(SmartDashboard.getNumber("stageTwoSetpoint",-152)));
    stageOneSub.setDesiredAngle(Units.degreesToRadians(SmartDashboard.getNumber("stageOneSetpoint",60)));
  }
}
