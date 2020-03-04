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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Autonomous;
import frc.robot.lib.InterpolatingDouble;

public class Robot extends TimedRobot {

  private Joystick stick = new Joystick(0);
  private Joystick jStick = new Joystick(1);
  private Compressor mCompressor = new Compressor(0);
  private double HoodPos = 0;
  private double ShooterRPM = 0;
  private double ShooterSpeed = 0;
  private boolean isInAttackMode = false;
  private Command m_autonomousCommand;
  private double lastTime = 0;

  public Robot(){
    super(Constants.kDt);
  }

  @Override
  public void robotInit() {
    mCompressor.start();
    Shooter.getInstance().zeroHood();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Shuffle.getInstance().Update();
    SmartDashboard.putNumber("Shooter RPM", Shooter.getInstance().getShooterRPM());
    double time = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("Loop Dt", (time - lastTime) * 1000);
    lastTime = time;
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
      double curDist = Limelight.getInstance().getDistance();
      double RPM = Constants.VISION.kRPMMap.getInterpolated(new InterpolatingDouble(curDist)).value;
      double hoodDist = Constants.VISION.kHoodMap.getInterpolated(new InterpolatingDouble(curDist)).value;
      Shooter.getInstance().setShooterRPM(RPM);
      Shooter.getInstance().setHoodPos(hoodDist);
      SmartDashboard.putNumber("Map RPM", RPM);
      SmartDashboard.putNumber("Map Hood Pos", hoodDist);
      SmartDashboard.putBoolean("In Range", Limelight.getInstance().inRange());
      if (Limelight.getInstance().inRange() && Math.abs(Shooter.getInstance().getShooterRPM() - RPM) < 25) {
        Elevator.getInstance().setElevatorOutput(.420);
        ElevatorStopper.getInstance().setStopper(true);
      }
      return;
    } else {
      double forward, turn, rightOut, leftOut;
      // this gets how far forward the forward stick is
      //forward = Math.pow(stick.getRawAxis(3) - stick.getRawAxis(2), 5) + Drive.getInstance().antiTip();
      //turn = Math.abs(stick.getRawAxis(0)) > .05 ? stick.getRawAxis(0) : 0.00; // this gets out left or right the turn stick is
     // rightOut = forward - turn; // This sets the turn distance for arcade drive
    //  leftOut = forward + turn;
      double antiTip = Drive.getInstance().antiTip();
       forward = (-stick.getRawAxis(2) + stick.getRawAxis(3)+ antiTip);
       turn = (-stick.getRawAxis(0));
      Drive.DriveSignal controlSig = cheesyDrive(forward, turn, true);
      leftOut = controlSig.getLeft();
      rightOut = controlSig.getRight();

      if (isInAttackMode) {
        Drive.getInstance().setOutput(new Drive.DriveSignal(leftOut / 2.0, rightOut / 2.0));
      } else {
        Drive.getInstance().setOutput(new Drive.DriveSignal(leftOut, rightOut));
      }
    }

    // Run Shooter
    /*
    if (jStick.getRawButton(1)) {
      Shooter.getInstance().setShooterOutput(.5);
    } else {
      Shooter.getInstance().setShooterOutput(0.00);
    }
    */

    if (jStick.getRawButtonPressed(Constants.INPUT.climbOn)) {
      Climber.getInstance().setClimbState(true);
    }
    if (jStick.getRawButtonPressed(Constants.INPUT.climbOff)) {
      Climber.getInstance().setClimbState(false);
    }

    if (jStick.getPOV() == 0) {
      HoodPos -= .05;
    } else if (jStick.getPOV() == 180) {
      HoodPos += .05;
    }
    if(jStick.getRawButtonPressed(6))
      ShooterSpeed += 50;
    else if(jStick.getRawButtonPressed(4))
      ShooterSpeed -= 50;
    
    Shooter.getInstance().setHoodPos(HoodPos);
    Shooter.getInstance().setShooterRPM(ShooterSpeed);
    ElevatorStopper.getInstance().setStopper(false);
    System.out.println("Target RPM: " + ShooterSpeed + " RPM: " + Shooter.getInstance().getShooterRPM() + " Hood Pos: " + Shooter.getInstance().getHoodPos());

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

    if(jStick.getRawButtonPressed(10)){
      ElevatorStopper.getInstance().toggleStopper();
    }

  }

  public void AttackMode() {
    Intake.getInstance().setIntakeRoller(.75);
    Intake.getInstance().setIntakeState(true);
    Elevator.getInstance().setElevatorOutput(0.20);
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
    System.out.println("RPM: " + Shooter.getInstance().getShooterRPM() + " Hood Pos: " + Shooter.getInstance().getHoodPos());
/*
    if (Shooter.getInstance().getShooterRPM() > TargetRPM) {
      Elevator.getInstance().setElevatorOutput(.75);
      Intake.getInstance().setHopperRoller(0.5);
      ElevatorStopper.getInstance().setStopper(false);
    } else {
      Elevator.getInstance().setElevatorOutput(.25);
      Intake.getInstance().setHopperRoller(0.5);
    }
    */
  }

  public Drive.DriveSignal cheesyDrive(double throttle, double wheel, boolean cubeInputs) {
    double kThrottleDeadband = 0.0;
    double kWheelDeadband = 0.003;
    double leftMotorSpeed;
    double rightMotorSpeed;
    double moveValue = limit(throttle);
    double rotateValue = limit(wheel);
    moveValue = handleDeadband(moveValue, kThrottleDeadband);
    rotateValue = handleDeadband(rotateValue, kWheelDeadband);
    if (cubeInputs) {
      rotateValue = rotateValue * rotateValue * rotateValue;
    }
    rotateValue = rotateValue / 2.3;
    if (moveValue > 0.0) {
      if (rotateValue > 0.0) {
        leftMotorSpeed = moveValue - rotateValue;
        rightMotorSpeed = Math.max(moveValue, rotateValue);
      } else {
        leftMotorSpeed = Math.max(moveValue, -rotateValue);
        rightMotorSpeed = moveValue + rotateValue;
      }
    } else {
      if (rotateValue > 0.0) {
        leftMotorSpeed = -Math.max(-moveValue, rotateValue);
        rightMotorSpeed = moveValue + rotateValue;
      } else {
        leftMotorSpeed = moveValue - rotateValue;
        rightMotorSpeed = -Math.max(-moveValue, -rotateValue);
      }
    }
    return new Drive.DriveSignal(leftMotorSpeed, rightMotorSpeed);
  }
    public double limit(double num) {
    if (num > 1.0) {
      return 1.0;
    }
    if (num < -1.0) {
      return -1.0;
    }
    return num;
  }

  public double handleDeadband(double val, double deadband) {
    return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
  }
}
