package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Intake.Intake;

public class Shooter extends SubsystemBase{
  private final CANSparkMax shooterLowerMotor = new CANSparkMax(ShooterConstants.shooterLowerMotorID, MotorType.kBrushless);
  private final CANSparkMax shooterUpperMotor = new CANSparkMax(ShooterConstants.shooterUpperMotorID, MotorType.kBrushless);
  private final Intake s_intake;

  public Shooter(Intake s_intake) {
    this.s_intake = s_intake;

    shooterLowerMotor.setInverted(true);
    shooterUpperMotor.setInverted(true);

    shooterLowerMotor.setIdleMode(IdleMode.kCoast);
    shooterUpperMotor.setIdleMode(IdleMode.kCoast);
  }

  public void moveShooterRollers(double velocity) {
    shooterLowerMotor.set(velocity);
    shooterUpperMotor.set(velocity);
  }

  /* 
   * To shoot a note in this robot, we need to spin both the shooter rollers and the
   * intake rollers, that's why in order for us to create an automatic function to
   * shoot a note in this subsystem, we need to call the intake subsystem.
  */
  public void shootAutomatically(Timer shooterTimer, double shooterVelocity, double intakeVelocity) {
    if (shooterTimer.get() == 0){
      shooterTimer.start();
    }
    else if (shooterTimer.get() <= 0.7){
      moveShooterRollers(shooterVelocity);
    }
    if (shooterTimer.get() >= 0.4 & shooterTimer.get() <= 0.6){
      s_intake.rollIntake(intakeVelocity, false, false);
    }
    else if (shooterTimer.get() >= 0.7){
      s_intake.stopIntakeRollers();
      stopShooter();
    }
  }

  public void stopShooter() {
    shooterLowerMotor.set(0);
    shooterUpperMotor.set(0);
  }

  @Override
  public void periodic() {}
}