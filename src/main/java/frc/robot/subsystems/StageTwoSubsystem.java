// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("stageTwoAngle", getAngle());
  }
  public void setPower(double power) {
    armMotorPrimary.set(power);
  }

  public double getAngle(){
    return encoder.getPosition() - Units.degreesToRadians(180);
    //return 0;
  }
    // This method will be called once per scheduler run
  }

