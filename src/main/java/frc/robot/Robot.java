/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

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

  double forward;
  double turn;

  double leftOut;
  double rightOut;

  static double lastTime;

  boolean zero = true;

  boolean liftSwitch = true;
  public boolean intakeRollSwitch = false;
  public double elevatorSwitch = 0;

  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(1.75, 0.75);

  private final ProfiledPIDController m_controller = new ProfiledPIDController(Constants.kP, 0.0, Constants.kD,
      constraints, 0.02);

  @Override
  public void robotInit() {
    Constants.rightMaster.setInverted(true);
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

  public void input() {
    forward = -Constants.stick.getRawAxis(3) + Constants.stick.getRawAxis(2); // this gets how far forward the forward
                                                                              // stick is
    turn = Constants.stick.getRawAxis(4); // this gets out left or right the turn stick is

    rightOut = forward - turn; // This sets the turn distance for arcade drive
    leftOut = forward + turn;

    if (Constants.jStick.getRawButtonReleased(1)) {
      intakeRollSwitch = !intakeRollSwitch;
    }

  }

  @Override
  public void teleopPeriodic() {
    input();

    Constants.rightMaster.set(ControlMode.PercentOutput, rightOut);
    Constants.leftMaster.set(ControlMode.PercentOutput, leftOut);

    Constants.rightSlave.follow(Constants.rightMaster);
    Constants.leftSlave.follow(Constants.leftMaster);

    if (Constants.stick.getRawButtonReleased(1)) {
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
