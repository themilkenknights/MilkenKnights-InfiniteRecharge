/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Autonomous;
import frc.robot.lib.InterpolatingDouble;
import frc.robot.lib.PressureSensor;

public class Robot extends TimedRobot {

  private Joystick stick = new Joystick(0);
  private Joystick jStick = new Joystick(1);
  private Compressor mCompressor = new Compressor(0);
  private PressureSensor pressureSensor = new PressureSensor(1);
  private double HoodPos = 0;
  private double ShooterRPM = 0;
  private double ShooterSpeed = 0;
  private boolean isInAttackMode = false;
  private Command m_autonomousCommand;

  @Override
  public void robotInit() {
    mCompressor.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Shuffle.getInstance().Update();
    SmartDashboard.putNumber("Shooter RPM", Shooter.getInstance().getShooterRPM());
    SmartDashboard.putNumber("Pressure Sensor Voltage", pressureSensor.getVoltage());
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = new Autonomous();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    Drive.getInstance().resetNavX();
    Shooter.getInstance().zeroHood();
    HoodPos = 0;
    ShooterRPM = 0;
    ShooterSpeed = 0;
  }

  @Override
  public void teleopPeriodic() {
    input();
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  public void input() {
    if (jStick.getRawButton(Constants.INPUT.limeLight)) {
      Drive.getInstance().setOutput(Limelight.getInstance().update());
      // Always Change the RPM of shooter while button is pressed so once we are in
      // range, it is
      // ready
      double curDist = Limelight.getInstance().getDistance();
      double RPM = Constants.VISION.kRPMMap.getInterpolated(new InterpolatingDouble(curDist)).value;
      Shooter.getInstance().setShooterRPM(RPM);
      Shooter.getInstance().setHoodPos(curDist);
      if (Limelight.getInstance().inRange()) {
        Elevator.getInstance().setElevatorOutput(.420);
        ElevatorStopper.getInstance().setStopper(true);
      }
    } else {
      double forward, turn, rightOut, leftOut;
      // this gets how far forward the forward stick is
      forward = Math.pow(stick.getRawAxis(3) - stick.getRawAxis(2), 5) + Drive.getInstance().antiTip();
      turn = stick.getRawAxis(0); // this gets out left or right the turn stick is
      rightOut = forward - turn; // This sets the turn distance for arcade drive
      leftOut = forward + turn;

      if (isInAttackMode) {
        Drive.getInstance().setOutput(new Drive.DriveSignal(leftOut / 2, rightOut / 2));
      } else {
        Drive.getInstance().setOutput(new Drive.DriveSignal(leftOut, rightOut));
      }
    }

    // Run Shooter
    if (jStick.getRawButton(1)) {
      Shooter.getInstance().setShooterOutput(.69);
    } else {
      Shooter.getInstance().setShooterOutput(0.00);
    }

    if (jStick.getRawButtonPressed(Constants.INPUT.climbOn)) {
      Climber.getInstance().setClimbState(true);
    }
    if (jStick.getRawButtonPressed(Constants.INPUT.climbOff)) {
      Climber.getInstance().setClimbState(false);
    }

    if (jStick.getPOV() == 0) {
      HoodPos -= .1;
    } else if (jStick.getPOV() == 180) {
      HoodPos += .1;
    }

    if (jStick.getRawButton(Constants.INPUT.attackMode)) {
      AttackMode();
    } else if (jStick.getRawButton(Constants.INPUT.defenceMode)) {
      DefenceMode();
    }

    if (jStick.getRawButton(Constants.INPUT.elevatorUp)) {
      Elevator.getInstance().setElevatorOutput(.420);
      ElevatorStopper.getInstance().setStopper(true);
    } else if (jStick.getRawButton(Constants.INPUT.elevatorDown)) {
      Elevator.getInstance().setElevatorOutput(-.420);
      ElevatorStopper.getInstance().setStopper(true);
    } else if (!isInAttackMode) {
      Elevator.getInstance().setElevatorOutput(0);
    }

    Shooter.getInstance().setHoodPos(HoodPos);
  }

  public void AttackMode() {
    Intake.getInstance().setIntakeRoller(.75);
    Intake.getInstance().setIntakeState(true);
    Elevator.getInstance().setElevatorOutput(.420);
    Intake.getInstance().setHopperRoller(.42);
    ElevatorStopper.getInstance().setStopper(false);
    isInAttackMode = true;
  }

  public void DefenceMode() {
    Intake.getInstance().setIntakeRoller(0.0);
    Intake.getInstance().setIntakeState(false);
    Elevator.getInstance().setElevatorOutput(0);
    isInAttackMode = false;
  }

  public void shoot(double TargetRPM, double HoodPos) {
    Shooter.getInstance().setShooterOutput(TargetRPM);
    Shooter.getInstance().setHoodPos(HoodPos);
    Shooter.getInstance().setShooterOutput(ShooterRPM);
    System.out
        .println("RPM: " + Shooter.getInstance().getShooterRPM() + " Hood Pos: " + Shooter.getInstance().getHoodPos());

    if (Shooter.getInstance().getShooterRPM() > TargetRPM) {
      Elevator.getInstance().setElevatorOutput(.75);
      Intake.getInstance().setHopperRoller(0.5);
      ElevatorStopper.getInstance().setStopper(false);
    } else {
      Elevator.getInstance().setElevatorOutput(.25);
      Intake.getInstance().setHopperRoller(0.5);
    }
  }
}
