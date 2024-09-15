// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakePivotAutomatically;
import frc.robot.commands.IntakeRollersMoveManually;
import frc.robot.commands.ShooterCommands.ShootNote;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;

public class RobotContainer {
  private static final Drive drive = new Drive();
  private static final Intake intake = new Intake();
  
  // This subsystem needs the intake subsystem to be able to work
  private static final Shooter shooter = new Shooter(intake);

  // private final Alert driverDisconnected = new Alert("Driver controller disconnected (this is a test).", AlertType.WARNING);
  // private final Alert testing = new Alert("testing this", AlertType.ERROR);

  // private final Alert driverDisconnected = new Alert();

  public RobotContainer() {

    drive.setDefaultCommand(
      new DriveCommand(
        drive,
        () -> (-driveControl.getRawAxis(1)),
        () -> (driveControl.getRawAxis(0)),
        () -> (-driveControl.getRawAxis(2))
      )
    );
    configureBindings();

    /*
    driverDisconnected.set(true);
    testing.set(true);
  */
  }


  private static final PS4Controller driveControl = new PS4Controller(0);
  public static final PS4Controller mecanismsControl = new PS4Controller(1);

  private void configureBindings() {
    // Chassis driver controls
    new JoystickButton(driveControl, IOConstants.buttonTriangle).whileTrue(new RunCommand(drive::zeroHeading));
    new JoystickButton(driveControl, IOConstants.triggerRight).whileTrue(new IntakePivotAutomatically(intake, 1)); //Floor
    new JoystickButton(driveControl, IOConstants.triggerRight).whileFalse(new IntakePivotAutomatically(intake, 3)); //Shooter

    // Mechanisms driver controls
    new JoystickButton(mecanismsControl, IOConstants.triggerRight).whileTrue(new IntakeRollersMoveManually(intake, IntakeConstants.intakeRollersMotorVelocityThrow));
    new JoystickButton(mecanismsControl, IOConstants.triggerLeft).whileTrue(new IntakeRollersMoveManually(intake, IntakeConstants.intakeRollersMotorVelocitySuck));
    
    new POVButton(mecanismsControl, IOConstants.arrowLeft).whileTrue(new IntakePivotAutomatically(intake, 1)); //Floor
    new JoystickButton(mecanismsControl, IOConstants.bumperRight).toggleOnTrue(new IntakePivotAutomatically(intake, 2)); //Amp    
    new JoystickButton(mecanismsControl, IOConstants.bumoerLeft).toggleOnTrue(new IntakePivotAutomatically(intake, 3)); //Shooter
    new POVButton(mecanismsControl, IOConstants.arrowLeft).whileFalse(new IntakePivotAutomatically(intake, 3)); //Shooter

    new JoystickButton(mecanismsControl, IOConstants.buttonTriangle).whileTrue(new ShootNote(shooter, intake));

  }

  public Command getAutonomousCommand() {
    // Reads the information sent from the auto chooser
    return null;
  }

  public Drive getChasisSubsystem() {
    return drive;
  }
}
