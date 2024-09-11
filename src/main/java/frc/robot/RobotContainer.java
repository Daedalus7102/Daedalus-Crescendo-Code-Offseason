// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Drive.Drive;

public class RobotContainer {
  private static final Drive drive = new Drive();
  // private final Alert driverDisconnected = new Alert();

  public RobotContainer() {

    drive.setDefaultCommand(
      new DriveCommand(
        drive,
        () -> (-driveControl.getRawAxis(1)),
        () -> (driveControl.getRawAxis(0)),
        () -> (-driveControl.getRawAxis(2))
      )
    );
    configureBindings();
  }


  private static final PS4Controller driveControl = new PS4Controller(0);
  public static final PS4Controller mecanismsControl = new PS4Controller(1);



  private void configureBindings() {
    // Chassis driver controls
    new JoystickButton(driveControl, Constants.IOConstants.buttonTriangle).whileTrue(new RunCommand(drive::zeroHeading));
  }

  public Command getAutonomousCommand() {
    // Reads the information sent from the auto chooser
    return null;
  }

  public Drive getChasisSubsystem() {
    return drive;
  }
}
