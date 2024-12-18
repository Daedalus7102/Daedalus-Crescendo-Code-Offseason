package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.Drive.Drive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class DriveCommand extends Command{
    Drive drive;
    Supplier<Double> xSpeed, ySpeed, zSpeed;
    private final PIDController xPID, yPID, zPID;
    // private final SlewRateLimiter xLimiter, yLimiter, zLimiter;
    private boolean robotRelative = false;

    public DriveCommand(Drive drive, Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> zSpeed, boolean robotRelative){
        addRequirements(drive);
        this.drive = drive;

        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.zSpeed = zSpeed;
        this.robotRelative = robotRelative;

        this.xPID = new PIDController(0.5, 0, 0);
        this.yPID = new PIDController(0.5, 0, 0);
        this.zPID = new PIDController(0.5, 0, 0);

        /*
        this.xLimiter = new SlewRateLimiter(SwerveDriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(SwerveDriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.zLimiter = new SlewRateLimiter(SwerveDriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        */
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
        xNeed = xPID.calculate(xNeed);
        yNeed = yPID.calculate(yNeed);
        zNeed = zPID.calculate(zNeed);

        /*
        xNeed = xLimiter.calculate(xNeed) * SwerveDriveConstants.kTeleDriveMaxSpeedMetersPerSecond;; 
        yNeed = yLimiter.calculate(yNeed) * SwerveDriveConstants.kTeleDriveMaxSpeedMetersPerSecond;; 
        zNeed = zLimiter.calculate(zNeed) * SwerveDriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;;
        */
        
        drive.setChassisSpeeds(xNeed, yNeed, zNeed, robotRelative);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
