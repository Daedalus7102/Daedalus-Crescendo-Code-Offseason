package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{
  private final CANSparkMax intakePivotMotor = new CANSparkMax(IntakeConstants.intakePivotMotorID, MotorType.kBrushless);//9
  private final CANSparkMax intakeRollersMotor = new CANSparkMax(IntakeConstants.intakeRollersMotorID, MotorType.kBrushless);//10
    
  private final DigitalInput infraredSensor =  new DigitalInput(IntakeConstants.infraredSensorChannel);

  private final CANcoder pivotCANCoder = new CANcoder(IntakeConstants.pivotCANcoderID, "Drivetrain");
  private final PIDController pivotPID = new PIDController(IntakeConstants.Intake_kP, IntakeConstants.Intake_kI, IntakeConstants.Intake_kD);

  private double goal; 
  private double PIDvalue;
  public String goalIntakePosition;
  private double intakeAngle;


  public Intake() {
    intakeRollersMotor.restoreFactoryDefaults();
    intakePivotMotor.restoreFactoryDefaults();

    intakeRollersMotor.setInverted(true);
    intakePivotMotor.setInverted(true);

    intakeRollersMotor.setIdleMode(IdleMode.kCoast);
    intakePivotMotor.setIdleMode(IdleMode.kBrake);
  }

  public void pivotMotorCoast() {
    intakePivotMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }
  public void pivotMotorBreak() {
    intakePivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  public boolean getInfraredSensorValue(){
    return !infraredSensor.get();
  }

  //Security method to avoid the intake from lowering more than expected
  public double getIntakeEncoderPosition() {
    intakeAngle = pivotCANCoder.getAbsolutePosition().getValue()*360 + IntakeConstants.intakeOffset; 
    if (intakeAngle >= 310 && intakeAngle <= 360){
      intakeAngle = 0;
    }
    return intakeAngle; 
  }


  public void setIntakePivotPosition(int position) {
    switch (position) {
      case 1:
        goal = IntakeConstants.floorGoalPosition;
        goalIntakePosition = "Floor";

        PIDvalue = pivotPID.calculate(getIntakeEncoderPosition(), goal);
        PIDvalue = desaturatePIDValue(PIDvalue);
        intakePivotMotor.set(PIDvalue);
        break;
      case 2:
        goal = IntakeConstants.ampGoalPosition;
        goalIntakePosition = "Amp";

        PIDvalue = pivotPID.calculate(getIntakeEncoderPosition(), goal);
        PIDvalue = desaturatePIDValue(PIDvalue);
        intakePivotMotor.set(PIDvalue);
        break;
      case 3:
        goal = IntakeConstants.shooterGoalPosition;
        goalIntakePosition = "Shooter";

        PIDvalue = pivotPID.calculate(getIntakeEncoderPosition(), goal);
        PIDvalue = desaturatePIDValue(PIDvalue);
        intakePivotMotor.set(PIDvalue); 
        break;
    }   
}

  public void stopIntakePivotMotor(){
    intakePivotMotor.set(0);
  }

  public void moveIntakeRollers(double velocity){
    intakeRollersMotor.set(velocity);
  }

  public void stopIntakeRollers(){
    intakeRollersMotor.set(0);
  }

  public double getIntakeRollersVelocity(){
    return intakeRollersMotor.getEncoder().getVelocity();
  }

  // Must indicate the parameter which gives the option to follow the feedback from the IR sensor
  public void rollIntake(double intakeRollersVelocity, boolean setAutomaticRollingIntake, boolean activateSecutirySystem){
    if (setAutomaticRollingIntake == true && goalIntakePosition == "Floor" && getIntakeEncoderPosition() <= 35 && getIntakeEncoderPosition() >= 11){
      moveIntakeRollers(intakeRollersVelocity);
    }
    else{
      //moveIntakeRollers(intakeRollersVelocity);
    }
  }
    
  public double desaturatePIDValue(double s_PIDvalue) {
    if (s_PIDvalue > IntakeConstants.intakePivotMotorMaxOutput) {
      s_PIDvalue = IntakeConstants.intakePivotMotorMaxOutput;
    }
    else if (s_PIDvalue < -IntakeConstants.intakePivotMotorMaxOutput) {
      s_PIDvalue = -IntakeConstants.intakePivotMotorMaxOutput;
    }
    return s_PIDvalue;
  }

  public void periodic() {
    // 
    SmartDashboard.putNumber("Intake rollers velocity", intakeRollersMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Pivot motor output current", intakePivotMotor.getOutputCurrent());

    SmartDashboard.putBoolean("Note detected", getInfraredSensorValue());
    SmartDashboard.putNumber("Pivot angle", getIntakeEncoderPosition());

    SmartDashboard.putBoolean("condition", goalIntakePosition == "Floor" && getIntakeEncoderPosition() <= 35 && getIntakeEncoderPosition() >= 11);
  }
}
