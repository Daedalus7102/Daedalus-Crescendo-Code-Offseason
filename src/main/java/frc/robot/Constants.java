package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Constants {

    public static final class SwerveDriveConstants {
        //Front Left Module Information (Used in the "Chassi" class)
        public static final int driveMotorIDfrontLeft = 3;
        public static final int turnMotorIDfrontLeft = 1;
        public static final int cancoderIDfrontLeft = 1;
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 180;

        //Front Right Module Information (Used in the "Chassi" class)
        public static final int driveMotorIDfrontRight = 2;
        public static final int turnMotorIDfrontRight = 7;
        public static final int cancoderIDfrontRight = 2;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 180;

        //Back Left Module Information (Used in the "Chassi" class)
        public static final int driveMotorIDbackLeft = 8;
        public static final int turnMotorIDbackLeft = 5;
        public static final int cancoderIDbackLeft = 3;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = 180;

        //Back Right Module Information (Used in the "Chassi" class)
        public static final int driveMotorIDbackRight = 4;
        public static final int turnMotorIDbackRight = 6;
        public static final int cancoderIDbackRight = 4;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 180;

        //PID values [We will assume that we have to use the same value for all 4 modules] (Used in "Chassis" class)
        public static final double genericModulekP = 0.0048;
        public static final double genericModulekI = 0.0;
        public static final double genericModulekD = 0.0;

        //Information used to know the distance that the chassi has moved
        public static final double driveRevsToMeters = 4 * Math.PI / (39.37 * 8.14)  * 1.25;
        public static final double driveRPS2MPS = driveRevsToMeters;

        //Variable setting the maximum speed of the modules (Used in "Chassi" class)
        public static final double chassisMaxOutput = 0.9;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 15;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;


        //Change these values accordinly to your needs (values for slew rate limiter)
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 1;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 2;

        public static final double kDeadband = 0.05;
    }

    public static final class IntakeConstants {
        public static final int intakePivotMotorID = 12;
        public static final int intakeRollersMotorID = 9;
        public static final int pivotCANcoderID = 5;

        public static final int infraredSensorChannel = 9;

        public static final double intakePivotMotorMaxOutput = 0.9;
        public static final double intakeRollersMotorVelocitySuck = -0.6;
        public static final double intakeRollersMotorVelocityThrow = 0.8;
        public static final double intakeRollersMotorVelocityThrowForShooter = 0.7;

        public static final double floorGoalPosition = 18; //1
        public static final double ampGoalPosition = 128; //2
        public static final double shooterGoalPosition = 181.8; //3
        public static final double Intake_kP = 0.0097;
        public static final double Intake_kI = 0;
        public static final double Intake_kD = 0;

        public static final double intakeOffset = 7;
        public static final double timeForIntaking = 0.8;
    }

    public static final class ShooterConstants {
        public static final int shooterLowerMotorID = 11;
        public static final int shooterUpperMotorID = 10;

        public static final double shooterMotorVelocity = 0.9;
    }

    public static final class ClimberConstants {
        public static final DoubleSolenoid.Value rise = DoubleSolenoid.Value.kForward;
        public static final DoubleSolenoid.Value lower = DoubleSolenoid.Value.kReverse;
        public static final DoubleSolenoid.Value stop = DoubleSolenoid.Value.kOff;
    }

    public static class AimbotConstants {
        public static double targetTXAimbotSpeaker = -8.41;
        public static double targetTYAimbotSpeaker = 1.63;

        public static final double xAprilTagThreshold = 5;
        public static final double yApriltagThreshold = 2;

        public static final double zDriveMaxSpeed = 0.8;
        public static final double yDriveMaxSpeed = 0.8;
        public static final double kPdriveY = 0.06;
        public static final double kPdriveZ = 0.24;
    }
}
