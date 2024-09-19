  // Copyright (c) FIRST and other WPILib contributors.
  // Open Source Software; you can modify and/or share it under the terms of
  // the WPILib BSD license file in the root directory of this project.

  package frc.robot.commands;
  import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Intake.PivotPosition;

  public class IntakePivotAutomatically extends Command {
    private final Intake s_intake;
    private final PivotPosition pivot_position;

    public IntakePivotAutomatically(Intake s_intake, PivotPosition pivot_position) {
      this.s_intake = s_intake;
      this.pivot_position = pivot_position;
      addRequirements(s_intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      s_intake.setIntakePivotPosition(pivot_position);
      s_intake.rollIntake(IntakeConstants.intakeRollersMotorVelocitySuck, true, true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      s_intake.stopIntakePivotMotor();
      s_intake.stopIntakeRollers();
      s_intake.timeForIntaking.reset();
    }

    @Override
    public boolean isFinished() {
      return false;
    }
  }