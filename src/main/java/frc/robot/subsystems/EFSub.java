// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class EFSub extends SubsystemBase {
  private TalonSRX efMotorTop;
  private TalonSRX efMotorBottom;

  private double[] cGCoordinateRelativeToPivot;
  private double mass;

  /** Creates a new EndEffectorSubsystem. */
  public EFSub() {
    efMotorTop = new TalonSRX(12);
    efMotorBottom = new TalonSRX(14);

    // setting motors to brake mode so they immediately stop moving when voltage stops being sent
    efMotorTop.setNeutralMode(NeutralMode.Coast);
    efMotorBottom.setNeutralMode(NeutralMode.Coast);

    efMotorBottom.setInverted(false);
    efMotorTop.setInverted(true);
    
    // limits power going to motor to prevent burnout
    // values need to be changed
    efMotorTop.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
      true, // enabled
      10, // Limit (amp)
      10, // Trigger Threshold (amp)
      0)); // Trigger Threshold Time(s)

    // limits power going to motor to prevent burnout
    // values need to be changed
    efMotorTop.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
      true, // enabled
      10, // Limit (amp)
      10, // Trigger Threshold (amp)
      0)); // Trigger Threshold Time(s)

  }


  public void driveThing(double input){
    if(Math.abs(input)<0.06){
      efMotorTop.set(TalonSRXControlMode.PercentOutput, -0.08);
      efMotorBottom.set(TalonSRXControlMode.PercentOutput, 0.08);
      return;
    }
    efMotorTop.set(TalonSRXControlMode.PercentOutput, input);
    efMotorBottom.set(TalonSRXControlMode.PercentOutput, -input);

  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
