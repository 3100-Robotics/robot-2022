package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AxisButton extends Trigger {

  GenericHID m_joystick;
  int m_axisNumber;
  private double THRESHOLD = 0.5;

  public AxisButton(GenericHID joystick, int axisNumber) {
    m_joystick = joystick;
    m_axisNumber = axisNumber;
  }

  public boolean get() {
    if (THRESHOLD < 0) {
      return m_joystick.getRawAxis(m_axisNumber) < THRESHOLD; // Return true if axis value is less than negative
                                                              // threshold
    } else {
      return m_joystick.getRawAxis(m_axisNumber) > THRESHOLD; // Return true if axis value is greater than positive
                                                              // threshold
    }
  }
}