package frc.robot.commands.ShooterCommands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AimbotConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;

public class AimbotCommand extends Command{
  private final Drive s_drive;
  private final Intake s_intake;
  private final Shooter s_shooter;

  private Timer shooterTimmer = new Timer();
  private boolean readyForShoot = false;

  public AimbotCommand(Drive s_drive, Shooter s_shooter, Intake s_intake) {
      this.s_drive = s_drive;
      this.s_shooter = s_shooter;
      this.s_intake = s_intake;
      addRequirements(s_drive);
    }
  
  @Override
  public void initialize() {}

  public double calibrateX(double tagPos) {
    boolean needsCalibration = tagPos <= AimbotConstants.targetTXAimbotSpeaker - AimbotConstants.xAprilTagThreshold 
                            || tagPos >= AimbotConstants.targetTXAimbotSpeaker + AimbotConstants.xAprilTagThreshold;

    if(needsCalibration){
      double error = (AimbotConstants.targetTXAimbotSpeaker) - tagPos; // Calculate error
      double speed = AimbotConstants.kPdriveZ * AimbotConstants.zDriveMaxSpeed * error / AimbotConstants.targetTXAimbotSpeaker; // Adjust velocity

      if(speed > AimbotConstants.zDriveMaxSpeed){
        speed = AimbotConstants.zDriveMaxSpeed;
      }
      return speed;
    }
    return 0.0;
  }

  public double calibrateY(double tagPos) {
    boolean needsCalibration = tagPos <= AimbotConstants.targetTYAimbotSpeaker - AimbotConstants.yApriltagThreshold 
                            || tagPos >= AimbotConstants.targetTYAimbotSpeaker + AimbotConstants.yApriltagThreshold;

    if (needsCalibration) {
      double error = AimbotConstants.targetTYAimbotSpeaker - tagPos; // Calculate error
      double speed = AimbotConstants.kPdriveY * AimbotConstants.yDriveMaxSpeed * error / AimbotConstants.targetTYAimbotSpeaker; // Adjust velocity
      
      if(speed > AimbotConstants.yDriveMaxSpeed){
        speed = AimbotConstants.yDriveMaxSpeed;
      }
      return speed;
    }
    return 0.0;
  }

  @Override
  public void execute() {    
    shooterTimmer.start();
    s_shooter.moveShooterRollers(-ShooterConstants.shooterMotorVelocity);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");

    // Read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double found = table.getEntry("tv").getDouble(0);

    if(found == 1){
        double zSpeed = calibrateX(x);
        double ySpeed = calibrateY(y);
        s_drive.setFieldOrientedSpeed(-ySpeed, 0, -zSpeed);

        if(Math.abs(x - AimbotConstants.targetTXAimbotSpeaker) <= AimbotConstants.xAprilTagThreshold
        && Math.abs(y - AimbotConstants.targetTYAimbotSpeaker) <= AimbotConstants.yApriltagThreshold
        && shooterTimmer.get() >= 0.4){
          s_intake.rollIntake(IntakeConstants.intakeRollersMotorVelocityThrowForShooter, false, false);
        }
        SmartDashboard.putBoolean("Ready for shoot", readyForShoot);
        SmartDashboard.putNumber("Required distance X", x - AimbotConstants.targetTXAimbotSpeaker);
        SmartDashboard.putNumber("Required distance Y", y - AimbotConstants.targetTYAimbotSpeaker);
    }
    else
    {
      s_drive.setFieldOrientedSpeed(0, 0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_drive.setFieldOrientedSpeed(0, 0, 0);
    s_shooter.stopShooter();
    s_intake.stopIntakeRollers();
    shooterTimmer.reset();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
