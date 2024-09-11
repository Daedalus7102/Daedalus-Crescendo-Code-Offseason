package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.Drive.Drive;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveCommand extends Command{
    Drive drive;
    Supplier<Double> xSpeed, ySpeed, zSpeed;
    private final SlewRateLimiter xLimiter, yLimiter, zLimiter;

    public DriveCommand(Drive drive, Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> zSpeed){
        addRequirements(drive);
        this.drive = drive;

        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.zSpeed = zSpeed;

        this.xLimiter = new SlewRateLimiter(SwerveDriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(SwerveDriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.zLimiter = new SlewRateLimiter(SwerveDriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xNeed = this.xSpeed.get();
        double yNeed = this.ySpeed.get();
        double zNeed = this.zSpeed.get();

        // 2. Apply deadband
        xNeed = Math.abs(xNeed) > SwerveDriveConstants.kDeadband ? xNeed : 0.0;
        yNeed = Math.abs(yNeed) > SwerveDriveConstants.kDeadband ? yNeed : 0.0;
        zNeed = Math.abs(zNeed) > SwerveDriveConstants.kDeadband ? zNeed : 0.0;

        // 3. Make the driving smoother
        xNeed = xLimiter.calculate(xNeed); 
        yNeed = yLimiter.calculate(yNeed);
        zNeed = zLimiter.calculate(zNeed);
        
        drive.setFieldOrientedSpeed(xNeed, yNeed, zNeed);

        SmartDashboard.putNumber("xSpeed", xNeed);
        SmartDashboard.putNumber("ySpeed", yNeed);
        SmartDashboard.putNumber("zSpeed", zNeed);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
