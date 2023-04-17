// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase {
  /** Creates a new GrabberSubsystem. */
  private StageOneSubsystem stageOneSub;
  private StageTwoSubsystem stageTwoSub;
  public GrabberSubsystem() {
    stageOneSub = new StageOneSubsystem();
    stageTwoSub = new StageTwoSubsystem();
  }

    public void joyStickDrive(double stageOnePow, double stageTwoPow) {
      stageOneSub.setPower(stageOnePow);
      stageTwoSub.setPower(stageTwoPow);
    }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
