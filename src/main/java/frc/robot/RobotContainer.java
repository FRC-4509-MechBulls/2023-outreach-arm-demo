// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.EFSub;
import frc.robot.subsystems.GrabberSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.nav.EFPathingTelemetrySub;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  EFPathingTelemetrySub efPathingTelemetrySub = new EFPathingTelemetrySub();
  GrabberSubsystem grabberSub = new GrabberSubsystem(efPathingTelemetrySub);
  SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  EFSub efSub = new EFSub();

  Joystick operator1 = new Joystick(0);
  Joystick operator2 = new Joystick(1);
  XboxController xboxController = new XboxController(2);
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    grabberSub.setDefaultCommand(new RunCommand(()-> grabberSub.joystickDrive(xboxController.getLeftY(),xboxController.getRightY(), xboxController.getLeftBumper()), grabberSub));
    efSub.setDefaultCommand(new RunCommand(()->efSub.driveThing(xboxController.getLeftTriggerAxis() - xboxController.getRightTriggerAxis()),efSub));
    swerveSubsystem.setDefaultCommand(new RunCommand(()->swerveSubsystem.joystickDrive(0,0,0),swerveSubsystem));
  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    new JoystickButton(xboxController, XboxController.Button.kA.value).whileTrue(new RunCommand(()->grabberSub.joystickDrive(operator1.getY()*0.5,operator2.getY()*0.5, xboxController.getLeftBumper()),grabberSub));
    new JoystickButton(xboxController,XboxController.Button.kB.value).whileTrue(new InstantCommand(()->grabberSub.goHome()));

    new JoystickButton(xboxController,XboxController.Button.kLeftBumper.value).whileTrue(new RunCommand(()->swerveSubsystem.joystickDrive(xboxController.getLeftY()*-1, xboxController.getLeftX()*-1, xboxController.getRightX()*-1),swerveSubsystem));
   // new JoystickButton(xboxController,XboxController.Button.kLeftBumper.value).whileTrue(new RunCommand(()->grabberSub.goHome(),grabberSub));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
