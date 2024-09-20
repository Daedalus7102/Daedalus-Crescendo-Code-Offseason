// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage

package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Interface for physical override switches on operator console. */
public class OverrideSwitches {
  private final GenericHID joystick;

  public OverrideSwitches(int port) {
    joystick = new GenericHID(port);
  }

  /** Returns whether the controller is connected. */
  //
  //
  public boolean isConnected() {
    return joystick.isConnected()
        && !DriverStation.getJoystickIsXbox(joystick.getPort())
        && joystick.getName().equals("Generic   USB  Joystick");
  }

  /** Gets the state of a driver-side switch (0-2 from left to right). */
  public boolean getDriverSwitch(int index) {
    if (index < 0 || index > 2) {
      throw new RuntimeException(
          "Invalid driver override index " + Integer.toString(index) + ". Must be 0-2.");
    }
    return joystick.getRawButton(index + 1);
  }

  /** Returns a trigger for a driver-side switch (0-2 from left to right). */
  public Trigger driverSwitch(int index) {
    return new Trigger(() -> getDriverSwitch(index));
  }
}