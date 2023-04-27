// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ArmConstants.*;

public class StageTwoSubsystem extends SubsystemBase {
  /** Creates a new StageTwoSubsystem. */
  private CANSparkMax armMotorPrimary;
  private CANSparkMax armMotorSecondary;
  private AbsoluteEncoder encoder;
  private PIDController pidController = new PIDController(1, 0,0);


public StageTwoSubsystem(){
  armMotorPrimary = new CANSparkMax(stageTwoLeftId, MotorType.kBrushless);
  armMotorSecondary = new CANSparkMax(stageTwoRightId,  MotorType.kBrushless);
  
  armMotorPrimary.restoreFactoryDefaults();
  armMotorSecondary.restoreFactoryDefaults();

  armMotorPrimary.enableVoltageCompensation(12.0);
  armMotorSecondary.enableVoltageCompensation(12.0);

  armMotorPrimary.setSecondaryCurrentLimit(stageTwoSecondaryCurrentLimit);
  armMotorPrimary.setSmartCurrentLimit(stageTwoSmartCurrentLimit);
  armMotorSecondary.setSecondaryCurrentLimit(stageTwoSecondaryCurrentLimit);
  armMotorSecondary.setSmartCurrentLimit(stageTwoSmartCurrentLimit);
  armMotorPrimary.setIdleMode(CANSparkMax.IdleMode.kCoast);
  armMotorSecondary.setIdleMode(CANSparkMax.IdleMode.kCoast);

  armMotorSecondary.follow(armMotorPrimary, true);

  encoder = armMotorPrimary.getAbsoluteEncoder(Type.kDutyCycle);
  configEncoder();
}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("stageTwoAngle", Units.radiansToDegrees(getAngle()));
    double stageTwoOut = pidController.calculate(getAngle());
    double maxOut = 0.3;
    stageTwoOut = Math.min(stageTwoOut,maxOut);
    stageTwoOut = Math.max(stageTwoOut,-maxOut);
    stageTwoOut +=aff;
    SmartDashboard.putNumber("stageTwoPIDOut", stageTwoOut);
    armMotorPrimary.set(stageTwoOut);
  }

  public void setDesiredAngle(double setpoint){
      pidController.setSetpoint(setpoint);
  }

  private double aff = 0;
  public void setAff(double aff){
    this.aff = aff;
  }

  public double getAngle(){
    return (encoder.getPosition() - Units.degreesToRadians(180))%(Math.PI*2);
    //return 0;
  }
  private void configEncoder() {
    encoder.setPositionConversionFactor((2 * Math.PI) /stageTwoEncoderRatio);
    encoder.setVelocityConversionFactor(((2 * Math.PI) / stageTwoEncoderRatio) / 60);
    encoder.setInverted(true);
    encoder.setZeroOffset(stageTwoEncoderOffset);
  }

    // This method will be called once per scheduler run
  }

