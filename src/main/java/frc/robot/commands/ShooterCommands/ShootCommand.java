// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class ShootCommand extends Command {
  private final Shooter s_shooter;
  private final Intake s_intake;
  private Timer shooterTimer = new Timer();

  public ShootCommand(Shooter s_shooter, Intake s_intake) {
    this.s_shooter = s_shooter;
    this.s_intake = s_intake;
    addRequirements(s_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_shooter.shootAutomatically(shooterTimer, ShooterConstants.shooterMotorVelocity, IntakeConstants.intakeRollersMotorVelocityThrowForShooter);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_shooter.stopShooter();
    s_intake.stopIntakeRollers();
    shooterTimer.reset();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
