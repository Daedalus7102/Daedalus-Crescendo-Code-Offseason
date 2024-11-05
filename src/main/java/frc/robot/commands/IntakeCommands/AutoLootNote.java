package frc.robot.commands.IntakeCommands;

import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoLootConstants;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Intake.Intake;

public class AutoLootNote extends Command{
  private final Drive s_drive;
  private final Intake s_intake;
  Supplier<Double> ySpeedSupplier, xSpeedSupplier;
  
  public AutoLootNote(Drive s_drive, Intake s_intake, Supplier<Double> ySpeedSupplier, Supplier<Double> xSpeedSupplier) {
      this.s_drive = s_drive;
      this.s_intake = s_intake;
      this.ySpeedSupplier = ySpeedSupplier;
      this.xSpeedSupplier = xSpeedSupplier;
      addRequirements(s_drive);
    }
  
  @Override
  public void initialize() {}

  public void lootNote(double notePos, Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier){
    double zSpeed = 0;
    double xSpeed = xSpeedSupplier.get();
    double ySpeed = ySpeedSupplier.get();

    boolean needsXCalibration = notePos <= AutoLootConstants.targetTXAutoLoot - AutoLootConstants.xNoteThreshold 
                            || notePos >= AutoLootConstants.targetTXAutoLoot + AutoLootConstants.xNoteThreshold;

    if(needsXCalibration){
      double error = (AutoLootConstants.targetTXAutoLoot) - notePos; // Calculate error
      zSpeed = AutoLootConstants.kPdriveZLoot * AutoLootConstants.zDriveMaxSpeedLoot * error / AutoLootConstants.targetTXAutoLoot; // Adjust velocity
    }
    s_drive.setChassisSpeeds(ySpeed, xSpeed, zSpeed, true);
  }

  @Override
  public void execute() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-intake");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");

    // Read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double found = table.getEntry("tv").getDouble(0);

    if(found == 1){
      lootNote(x, xSpeedSupplier, ySpeedSupplier);
      SmartDashboard.putNumber("Required distance X", x - AutoLootConstants.targetTXAutoLoot);
    } else {
      s_drive.setChassisSpeeds(ySpeedSupplier.get(), xSpeedSupplier.get(), 0, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_drive.setChassisSpeeds(0, 0, 0, true);
    s_intake.stopIntakeRollers();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
