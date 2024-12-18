// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.ClimberCommands.ClimbCommand;
import frc.robot.commands.IntakeCommands.AutoLootNote;
import frc.robot.commands.IntakeCommands.IntakePivotAutomatically;
import frc.robot.commands.IntakeCommands.IntakeRollersMoveManually;
import frc.robot.commands.ShooterCommands.AimbotCommand;
import frc.robot.commands.ShooterCommands.ShootCommand;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Intake.PivotPosition;
import frc.robot.subsystems.Shooter.Shooter;

public class RobotContainer {
  // Subsystems
  public Drive drive = new Drive();
  public Intake intake = new Intake();
  public Shooter shooter = new Shooter(intake);
  public Climber climber = new Climber();

  // Controllers
  public static final CommandPS4Controller driverController = new CommandPS4Controller(0);
  public static final CommandPS4Controller operatorController = new CommandPS4Controller(1);
  private final SendableChooser<Command> autoChooser;
  
  public RobotContainer() {
    NamedCommands.registerCommand("SHOOT", new ShootCommand(shooter, intake).withTimeout(1.2));
    NamedCommands.registerCommand("LOWER_INTAKE", new IntakePivotAutomatically(intake, PivotPosition.FLOOR, false).withTimeout(2));
    NamedCommands.registerCommand("RISE_INTAKE", new IntakePivotAutomatically(intake, PivotPosition.SHOOTER, false).withTimeout(0.7));
    NamedCommands.registerCommand("AIMBOT", new AimbotCommand(drive, shooter, intake).withTimeout(4));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    // ----------- Driver Controller -----------
    /*
    drive.setDefaultCommand(
      new DriveCommand(
        drive,
        () -> (driverController.getLeftY()),
        () -> (-driverController.getLeftX()),
        () -> (driverController.getRightX()),
        false
      )
    );

    driverController.L1().whileTrue(
      new DriveCommand(
        drive,
        () -> (driverController.getLeftY()),
        () -> (-driverController.getLeftX()),
        () -> (driverController.getRightX()),
        true
      )
    );
    */

    driverController.triangle().whileTrue(new InstantCommand(() -> drive.zeroHeading()));
    driverController.R2().whileTrue(new IntakePivotAutomatically(intake, PivotPosition.FLOOR, true))
                        .whileFalse(new IntakePivotAutomatically(intake, PivotPosition.SHOOTER, false));

    driverController.cross().whileTrue(new AimbotCommand(drive, shooter, intake));
    driverController.circle().whileTrue(new AutoLootNote(drive, intake, () -> driverController.getLeftY(), () -> -driverController.getLeftX()));

    // ----------- Operator Controller -----------
    operatorController.R2().whileTrue(new IntakeRollersMoveManually(intake, IntakeConstants.intakeRollersMotorVelocityThrow));
    operatorController.L2().whileTrue(new IntakeRollersMoveManually(intake, IntakeConstants.intakeRollersMotorVelocitySuck));

    operatorController.povLeft().whileTrue(new IntakePivotAutomatically(intake, PivotPosition.FLOOR, true))
                                .whileFalse(new IntakePivotAutomatically(intake, PivotPosition.SHOOTER, false));

    operatorController.R1().toggleOnTrue(new IntakePivotAutomatically(intake, PivotPosition.AMP, false));
    operatorController.L1().toggleOnTrue(new IntakePivotAutomatically(intake, PivotPosition.SHOOTER, false));

    operatorController.triangle().toggleOnTrue(new ShootCommand(shooter, intake));

    operatorController.povUp().whileTrue(new ClimbCommand(climber, ClimberConstants.rise));
    operatorController.povDown().whileTrue(new ClimbCommand(climber, ClimberConstants.lower));
  }

  public Command getAutonomousCommand() {
    // Reads the information sent from the auto chooser
    return autoChooser.getSelected();
  }

  public Drive getChasisSubsystem() {
    return drive;
  }

  public Intake getIntakeSubsystem() {
    return intake;
  }
}
