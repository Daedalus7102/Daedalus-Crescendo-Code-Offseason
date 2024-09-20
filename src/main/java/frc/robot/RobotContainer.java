// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommands.IntakePivotAutomatically;
import frc.robot.commands.IntakeCommands.IntakeRollersMoveManually;
import frc.robot.commands.ShooterCommands.ShootNoteCommand;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Intake.PivotPosition;
import frc.robot.subsystems.Shooter.Shooter;

public class RobotContainer {
  // Subsystems
  public Drive drive = new Drive();
  public Intake intake = new Intake();
  public Shooter shooter = new Shooter(intake);

  // IntakePivotAutomatically intakeCommand = new IntakePivotAutomatically(intake, null);

  // Controllers
  public static final CommandPS4Controller driverController = new CommandPS4Controller(0);
  public static final CommandPS4Controller operatorController = new CommandPS4Controller(1);

  // private final Alert driverDisconnected = new Alert("Driver controller disconnected (this is a test).", AlertType.WARNING);
  // private final Alert testing = new Alert("testing this", AlertType.ERROR);
  // private final Alert driverDisconnected = new Alert();

  public RobotContainer() {
    configureBindings();

    /*
    driverDisconnected.set(true);
    testing.set(true);
  */
  }

  private void configureBindings() {
    // ----------- Driver Controller -----------
    drive.setDefaultCommand(
      new DriveCommand(
        drive,
        () -> (-driverController.getLeftX()),
        () -> (driverController.getLeftY()),
        () -> (-driverController.getRightX())
      )
    );

    // // driverController.cross().onTrue();

    driverController.triangle().whileTrue(new InstantCommand(() -> drive.zeroHeading()));
    driverController.R2().whileTrue(new IntakePivotAutomatically(intake, PivotPosition.FLOOR))
                        .whileFalse(new IntakePivotAutomatically(intake, PivotPosition.SHOOTER));

    
    // ----------- Operator Controller -----------
    operatorController.R2().whileTrue(new IntakeRollersMoveManually(intake, IntakeConstants.intakeRollersMotorVelocityThrow));
    operatorController.L2().whileTrue(new IntakeRollersMoveManually(intake, IntakeConstants.intakeRollersMotorVelocitySuck));

    operatorController.povLeft().whileTrue(new IntakePivotAutomatically(intake, PivotPosition.FLOOR))
                                .whileFalse(new IntakePivotAutomatically(intake, PivotPosition.SHOOTER));

    operatorController.R1().whileTrue(new IntakePivotAutomatically(intake, PivotPosition.AMP));
    operatorController.L1().whileTrue(new IntakePivotAutomatically(intake, PivotPosition.SHOOTER));

    operatorController.triangle().toggleOnTrue(new ShootNoteCommand(shooter, intake));

  }

  public Command getAutonomousCommand() {
    // Reads the information sent from the auto chooser
    return null;
  }

  public Drive getChasisSubsystem() {
    return drive;
  }
}
