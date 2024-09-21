// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ClimberCommands.ClimbCommand;
import frc.robot.commands.IntakeCommands.IntakePivotAutomatically;
import frc.robot.commands.IntakeCommands.IntakeRollersMoveManually;
import frc.robot.commands.ShooterCommands.AimbotCommand;
import frc.robot.commands.ShooterCommands.ShootCommand;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Intake.PivotPosition;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.utils.Alert;
import frc.robot.utils.OverrideSwitches;
import frc.robot.utils.Alert.AlertType;

public class RobotContainer {
  // Subsystems
  public Drive drive = new Drive();
  public Intake intake = new Intake();
  public Shooter shooter = new Shooter(intake);
  public Climber climber = new Climber();
  public OverrideSwitches overrides = new OverrideSwitches(5);

  // Controllers
  public static final CommandPS4Controller driverController = new CommandPS4Controller(0);
  public static final CommandPS4Controller operatorController = new CommandPS4Controller(1);
  // private final Trigger robotRelative = overrides.driverSwitch(0);
  boolean robotRelative = false;

  private final Alert driverControllerDisconnected = new Alert("Driver controller disconnected (Port 0).", AlertType.WARNING);
  private final Alert operatorControllerDisconnected = new Alert("Operator controller disconnected (Port 1).", AlertType.WARNING);
  
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // ----------- Driver Controller -----------
    drive.setDefaultCommand(
      new DriveCommand(
        drive,
        () -> (-driverController.getLeftY()),
        () -> (driverController.getLeftX()),
        () -> (-driverController.getRightX()),
        robotRelative
      )
    );

    driverController.L1().toggleOnTrue(
      new DriveCommand(
        drive,
        () -> (-driverController.getLeftY()),
        () -> (driverController.getLeftX()),
        () -> (-driverController.getRightX()),
        true
      )
    );

    driverController.triangle().whileTrue(new InstantCommand(() -> drive.zeroHeading()));
    driverController.R2().whileTrue(new IntakePivotAutomatically(intake, PivotPosition.FLOOR))
                        .whileFalse(new IntakePivotAutomatically(intake, PivotPosition.SHOOTER));

    driverController.cross().whileTrue(new AimbotCommand(drive, shooter, intake));
    
    // ----------- Operator Controller -----------
    operatorController.R2().whileTrue(new IntakeRollersMoveManually(intake, IntakeConstants.intakeRollersMotorVelocityThrow));
    operatorController.L2().whileTrue(new IntakeRollersMoveManually(intake, IntakeConstants.intakeRollersMotorVelocitySuck));

    operatorController.povLeft().whileTrue(new IntakePivotAutomatically(intake, PivotPosition.FLOOR))
                                .whileFalse(new IntakePivotAutomatically(intake, PivotPosition.SHOOTER));

    operatorController.R1().toggleOnTrue(new IntakePivotAutomatically(intake, PivotPosition.AMP));
    operatorController.L1().toggleOnTrue(new IntakePivotAutomatically(intake, PivotPosition.SHOOTER));

    operatorController.triangle().toggleOnTrue(new ShootCommand(shooter, intake));

    operatorController.povUp().whileTrue(new ClimbCommand(climber, ClimberConstants.rise));
    operatorController.povDown().whileTrue(new ClimbCommand(climber, ClimberConstants.lower));

    driverControllerDisconnected.set(!DriverStation.isJoystickConnected(driverController.getHID().getPort()));
    operatorControllerDisconnected.set(!DriverStation.isJoystickConnected(operatorController.getHID().getPort()));
  }

  public Command getAutonomousCommand() {
    // Reads the information sent from the auto chooser
    return null;
  }

  public Drive getChasisSubsystem() {
    return drive;
  }

  public Intake getIntakeSubsystem() {
    return intake;
  }
}
