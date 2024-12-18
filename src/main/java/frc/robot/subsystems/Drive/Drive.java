package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;

public class Drive extends SubsystemBase {

    private final Field2d field;
        // Specific and fixed values ​​that each module will have, such as the ID of its motors, its PID value, etc. The data is stored in the
        // class "Constants" and are being accessed from the nomenclature Constants.'variable name'
        private Module frontLeft = new Module(SwerveDriveConstants.driveMotorIDfrontLeft, 
                                    SwerveDriveConstants.turnMotorIDfrontLeft, 
                                    SwerveDriveConstants.cancoderIDfrontLeft, 
                                    SwerveDriveConstants.genericModulekP, 
                                    SwerveDriveConstants.genericModulekI, 
                                    SwerveDriveConstants.genericModulekD, 
                                    SwerveDriveConstants.FRONT_LEFT_MODULE_STEER_OFFSET, "Front Left");

        private Module frontRight = new Module(SwerveDriveConstants.driveMotorIDfrontRight, 
                                    SwerveDriveConstants.turnMotorIDfrontRight, 
                                    SwerveDriveConstants.cancoderIDfrontRight, 
                                    SwerveDriveConstants.genericModulekP, 
                                    SwerveDriveConstants.genericModulekI, 
                                    SwerveDriveConstants.genericModulekD,
                                    SwerveDriveConstants.FRONT_RIGHT_MODULE_STEER_OFFSET, "Front Right");

        private Module backLeft = new Module(SwerveDriveConstants.driveMotorIDbackLeft, 
                                    SwerveDriveConstants.turnMotorIDbackLeft, 
                                    SwerveDriveConstants.cancoderIDbackLeft, 
                                    SwerveDriveConstants.genericModulekP, 
                                    SwerveDriveConstants.genericModulekI, 
                                    SwerveDriveConstants.genericModulekD,
                                    SwerveDriveConstants.BACK_LEFT_MODULE_STEER_OFFSET, "Back Left");

        private Module backRight = new Module(SwerveDriveConstants.driveMotorIDbackRight, 
                                    SwerveDriveConstants.turnMotorIDbackRight, 
                                    SwerveDriveConstants.cancoderIDbackRight, 
                                    SwerveDriveConstants.genericModulekP, 
                                    SwerveDriveConstants.genericModulekI, 
                                    SwerveDriveConstants.genericModulekD,
                                    SwerveDriveConstants.BACK_RIGHT_MODULE_STEER_OFFSET, "Back Right");
    
    
    // "x" and "y" values ​​that represent the location of each module in the chassis
    Translation2d frontLeftTranslation = new Translation2d(Math.toRadians(-11.5), Math.toRadians(11.5)); //Units in Meters
    Translation2d frontRightTranslation = new Translation2d(Math.toRadians(11.5), Math.toRadians(11.5)); //Units in Meters
    Translation2d backLeftTranslation = new Translation2d(Math.toRadians(-11.5), Math.toRadians(-11.5)); //Units in Meters
    Translation2d backRightTranslation = new Translation2d(Math.toRadians(11.5), Math.toRadians(-11.5)); //Units in Meters
    // Declare Gyroscope Pigeon 2
    final Pigeon2 gyro = new Pigeon2(0, "Drivetrain");

    final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftTranslation, backLeftTranslation, frontRightTranslation, backRightTranslation);

    SwerveModulePosition[] positions = {frontLeft.getPosition(), backLeft.getPosition(), 
        frontRight.getPosition(), backRight.getPosition()};

    SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getRotation2d(), positions, new Pose2d(0, 0, getRotation2d()));

    SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, getRotation2d(), positions, new Pose2d(0, 0, getRotation2d()));

    // Method that would be used if we did not want the robot to have court orientation (this way does not implement the gyroscope)
    public void setRobotRelativeSpeeds(double xSpeed, double ySpeed, double zSpeed){
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, zSpeed));
    
        setModuleStates(states);
    }

    public void setFieldOrientedSpeed(double xSpeed, double ySpeed, double zSpeed){
        ChassisSpeeds chassisSpeedsFieldOriented = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, getRotation2d());    
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeedsFieldOriented);
        
        // Method that limits all modules proportionally to maintain the desired behavior when moving the robot
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveDriveConstants.chassisMaxOutput);
        setModuleStates(states);
    }

    public void setChassisSpeeds(double xSpeed, double ySpeed, double zSpeed, boolean robotRelative) {
        if (robotRelative == false) {
            setFieldOrientedSpeed(xSpeed, ySpeed, zSpeed);
        } else {
            setRobotRelativeSpeeds(xSpeed, ySpeed, zSpeed);
        }
    }

    public void setChassisToBreak(){
        frontLeft.swerveMotorToBreak();
        frontRight.swerveMotorToBreak();
        backLeft.swerveMotorToBreak();
        backRight.swerveMotorToBreak();
    }

    public void setChassisToCoast(){
        frontLeft.swerveMotorsToCoast();
        frontRight.swerveMotorsToCoast();
        backLeft.swerveMotorsToCoast();
        backRight.swerveMotorsToCoast();
    }

    private void setModuleStates(SwerveModuleState[] states){
        /* It is important to assign the values ​​of the array in this order because that is how it was
        declared each state separately and have to be assigned to the indicated one */
        frontLeft.setDesiredState(states[0], "Front Left");
        frontRight.setDesiredState(states[1], "Front Right");
        backLeft.setDesiredState(states[2], "Back Left");
        backRight.setDesiredState(states[3], "Back Right");
    }

    private double getAngle(){
        // Function to ask Pigeon the angle it is measuring
        return (this.gyro.getAngle())%360;
    }

    private ChassisSpeeds getChassisSpeeds() {
    return new ChassisSpeeds(0, 0, 0);
    }

    private void runVelcAuto(ChassisSpeeds speeds){
        setModuleStates(kinematics.toSwerveModuleStates(speeds));
    }

    // public ChassisSpeeds getFieldOrienteSpeeds(){
    //     return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getRotation2d());
    // }

    private Rotation2d getRotation2d(){
        return new Rotation2d(Math.toRadians(getAngle()));
    }

    public Pose2d getPose2d(){
        return odometry.getPoseMeters();
    }

    private void setOdoPose(Pose2d pose){
        positions[0] = frontLeft.getPosition();
        positions[1] = frontRight.getPosition();
        positions[2] = backLeft.getPosition();
        positions[3] = backRight.getPosition();

        odometry.resetPosition(getRotation2d(), positions, pose);
        poseEstimator.resetPosition(getRotation2d(), positions, pose);
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public Drive(){
        AutoBuilder.configureHolonomic(
            this::getPose2d, 
            this::setOdoPose, 
            this::getChassisSpeeds, 
            this::runVelcAuto, 
            new HolonomicPathFollowerConfig(
                new PIDConstants(2.7, 0, 0), 
                new PIDConstants(3, 0, 0), 
                4.8,
                0.46,
                new ReplanningConfig()
            ), 
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this);
        field = new Field2d();
    }

    public void updateShuffle(){
        field.setRobotPose(odometry.getPoseMeters());

        positions[0] = frontLeft.getPosition();
        positions[1] = frontRight.getPosition();
        positions[2] = backLeft.getPosition();
        positions[3] = backRight.getPosition();
    }

    @Override
    // The periodic works to see minimal things within the subsystem (It works even when it is disabled)
    public void periodic() {
        odometry.update(getRotation2d(), positions);
        poseEstimator.update(getRotation2d(), positions);
        
        SmartDashboard.putNumber("TurnMotor frontLeft angle", this.frontLeft.getTurnEncoder());
        SmartDashboard.putNumber("TurnMotor frontRigh anglet", this.frontRight.getTurnEncoder());
        SmartDashboard.putNumber("TurnMotor backLeft angle", this.backLeft.getTurnEncoder());
        SmartDashboard.putNumber("TurnMotor backright angle", this.backRight.getTurnEncoder());

        SmartDashboard.putNumber("DriveMotor frontLeft output", this.frontLeft.getDriveVelocity());
        SmartDashboard.putNumber("DriveMotor frontRigh output", this.frontRight.getDriveVelocity());
        SmartDashboard.putNumber("DriveMotor backLeft output", this.backLeft.getDriveVelocity());
        SmartDashboard.putNumber("DriveMotor backright output", this.backRight.getDriveVelocity());
        SmartDashboard.putNumber("Gyro value", getAngle());

        SmartDashboard.putData("Field", field);

        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        updateShuffle();
    }
}