// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static  frc.robot.Constants.ArmConstants.*;

public class StageOneSubsystem extends SubsystemBase {
  private TalonSRX armMotorPrimary;
  private TalonSRX armMotorSecondary;

  private PIDController pidController = new PIDController(2, 0,0);

  /** Creates a new StageOneSubsystem. */
  public StageOneSubsystem() {
    armMotorPrimary = new TalonSRX(stageOneRightId);
    armMotorSecondary = new TalonSRX(stageOneLeftId);

    armMotorPrimary.configFactoryDefault(1000);
    armMotorSecondary.configFactoryDefault(1000);

    armMotorPrimary.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0,1000);
    armMotorPrimary.configFeedbackNotContinuous(true,1000);

    armMotorPrimary.setNeutralMode(NeutralMode.Coast);
    //directionality
    armMotorPrimary.setSensorPhase(false);
    armMotorPrimary.setInverted(true);

    armMotorSecondary.follow(armMotorPrimary);
    armMotorSecondary.setInverted(InvertType.OpposeMaster);
    armMotorSecondary.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, continuousCurrentLimit, peakCurrentLimit, peakCurrentTime), 1000);
    armMotorSecondary.configVoltageCompSaturation(12.0, 1000);
    armMotorSecondary.enableVoltageCompensation(true);
    armMotorSecondary.setNeutralMode(NeutralMode.Coast);
  }

  public void setDesiredAngle(double setpoint){
    pidController.setSetpoint(setpoint);
  }

  double aff = 0;
  public void setAff(double aff){
    this.aff = aff;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("stageOneAngle", Units.radiansToDegrees(getAngle()));

    double stageOneOut = pidController.calculate(getAngle()) + aff;
    double maxOut = 0.50;
    stageOneOut = Math.min(stageOneOut,maxOut);
    stageOneOut = Math.max(stageOneOut,-maxOut);
    SmartDashboard.putNumber("stageOnePIDOut", stageOneOut);


    armMotorPrimary.set(TalonSRXControlMode.PercentOutput,stageOneOut);
  }
  public double getAngle(){
    return (armMotorPrimary.getSelectedSensorPosition()* stageOneEncoderTicksToRadians + stageOneEncoderOffset)%(Math.PI*2);
  }
}
