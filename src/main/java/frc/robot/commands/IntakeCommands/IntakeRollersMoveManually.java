// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

public class IntakeRollersMoveManually extends Command {

  private final Intake s_intake;
  private final double intake_velocity;

  public IntakeRollersMoveManually(Intake s_intake, double intake_velocity) {
    this.s_intake = s_intake;
    this.intake_velocity = intake_velocity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_intake.rollIntake(intake_velocity, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_intake.stopIntakeRollers();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
