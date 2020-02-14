/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private double forward;
  private double turn;

  private double leftOut;
  private double rightOut;

  private double lastTime;

  private boolean zero = true;

  private boolean liftSwitch = true;
  private boolean intakeRollSwitch = false;
  private double elevatorSwitch = 0;

  private Joystick stick = new Joystick(0);
  private Joystick jStick = new Joystick(1);

  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1.75, 0.75);

  private final ProfiledPIDController m_controller = new ProfiledPIDController(Constants.kP, 0.0, Constants.kD,
      constraints, 0.02);

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
    Constants.time.start();
  }

  @Override
  public void teleopPeriodic() {
    input();

    if (stick.getRawButtonReleased(1)) {
      if (liftSwitch) {
        m_controller.setGoal(90);
        liftSwitch = !liftSwitch;
      } else {
        m_controller.setGoal(0);
        liftSwitch = !liftSwitch;
      }
    }
    Constants.lifter.set(ControlMode.PercentOutput, m_controller.calculate(getArmPosition()));

    if (!intakeRollSwitch) {
      Constants.intakeRoller.set(ControlMode.PercentOutput, 0);
    } else {
      Constants.intakeRoller.set(ControlMode.PercentOutput, 1);
    }

    update();
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  public void input() {
    if (jStick.getRawButtonReleased(1)) {
      intakeRollSwitch = !intakeRollSwitch;
    }

    if (jStick.getRawButton(2)) {
      Limelight.getInstance().update();
    } else {
      forward = -stick.getRawAxis(3) + stick.getRawAxis(2); // this gets how far forward the forward stick is
      turn = stick.getRawAxis(4); // this gets out left or right the turn stick is

      rightOut = forward - turn; // This sets the turn distance for arcade drive
      leftOut = forward + turn;
      Drive.getInstance().setPercentOutput(leftOut, rightOut);
    }
  }

  public void update() {
    lastTime = Constants.time.get();
  }

  public double getArmPosition() { // Returns Degrees
    return ((Constants.lifter.getSelectedSensorPosition()) / 4096.0) * 360.0;
  }

  public double getArmVelocity() { // Returns Degrees Per Second
    return ((Constants.lifter.getSelectedSensorVelocity() * 10) / 4096.0) * 360.0;
  }

}
