/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Lifter.LifterState;

public class Robot extends TimedRobot {

  private Joystick stick = new Joystick(0);
  private Joystick jStick = new Joystick(1);

  @Override
  public void robotInit() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {
    input();

    if (jStick.getRawButtonPressed(5)) {
      Lifter.getInstance().setLifterState(LifterState.POS1);
    } else if (jStick.getRawButtonPressed(6)) {
      Lifter.getInstance().setLifterState(LifterState.POS2);
    }

    if (jStick.getRawButton(19)) {
      IntakeRoller.getInstance().setIntakeRoller(0.3);
    } else {
      IntakeRoller.getInstance().setIntakeRoller(0.0);
    }
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  public void input() {
    if (jStick.getRawButton(2)) {
      Drive.getInstance().setOutput(Limelight.getInstance().update());
    } else {
      double forward, turn, rightOut, leftOut;
      forward = -stick.getRawAxis(3) + stick.getRawAxis(2); // this gets how far forward the forward stick is
      turn = stick.getRawAxis(4); // this gets out left or right the turn stick is
      rightOut = forward - turn; // This sets the turn distance for arcade drive
      leftOut = forward + turn;
      Drive.getInstance().setOutput(new Drive.DriveSignal(leftOut, rightOut));
    }
  }
}
