/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drive.DriveSignal;

public class Robot extends TimedRobot {

  private Joystick stick = new Joystick(0);
  private Joystick jStick = new Joystick(1);
  private Compressor mCompressor = new Compressor(0);
  private double HoodPos = 0;
  private double ShooterRPM = 0;
  private double ShooterSpeed = 0;

  private boolean isInAttackMode = false;

  public static Timer time = new Timer();
  public static double lastTime;
  double highestVel = 0;
  double highestAcc = 0;

  @Override
  public void robotInit() {
    mCompressor.start();
  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    time.start();

    Shooter.getInstance().zeroHood();
    HoodPos = 0;
    ShooterRPM = 0;

    ShooterSpeed = 0;

  }

  @Override
  public void teleopPeriodic() {
    input();
    Shuffle.getInstance().Update();
    /*
    vel =Drive.getInstance().getVelocity();
    acc =Drive.getInstance().getAcceleration();
    */

    SmartDashboard.putNumber("Shooter RPM", Shooter.getInstance().getShooterRPM());

    /*
    if (Math.abs(vel) > highestVel) {
      highestVel = Math.abs(vel);
      System.out.println("Highest Velocity: " + highestVel);
    }
    
    if (Math.abs(acc) > highestAcc) {
      highestAcc = Math.abs(acc);
      System.out.println("Highest Acceleration: " + highestAcc);
    }
    */

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
    }
    else {
      double forward, turn, rightOut, leftOut;
      forward = (Math.pow(stick.getRawAxis(3) - stick.getRawAxis(2), 5) + Drive.getInstance().AntiTip()); // this gets
      // how far forward the forward stick is
      turn = stick.getRawAxis(0); // this gets out left or right the turn stick is
      rightOut = forward - turn; // This sets the turn distance for arcade drive
      leftOut = forward + turn;

      if (isInAttackMode) {
        Drive.getInstance().setOutput(new Drive.DriveSignal(leftOut / 2, rightOut / 2));
      }
      else {
        Drive.getInstance().setOutput(new Drive.DriveSignal(leftOut, rightOut));
      }
    }

    // Run Shooter
    if (jStick.getRawButton(1))
      Shooter.getInstance().setShooterOutput(.55);
    else
      Shooter.getInstance().setShooterOutput(0.0);

    // Run Elevator Up and Down
    /*
    if (jStick.getRawButton(6)) {
      Elevator.getInstance().setElevatorOutput(0.75, .5);
    } else if (jStick.getRawButton(4)) {
      Elevator.getInstance().setElevatorOutput(-0.5, -.3);
    } else if (jStick.getRawButton(9)) {
      Elevator.getInstance().setElevatorOutput(-0.75, -.5);
    
    } else
      Elevator.getInstance().setElevatorOutput(0.00, 0.00);
    */

    if (jStick.getRawButtonPressed(Constants.INPUT.climbOn)) {
      Climber.setClimbState(true);
    }
    if (jStick.getRawButtonPressed(Constants.INPUT.climbOff)) {
      Climber.setClimbState(false);
    }

    if (jStick.getPOV() == 0) {
      HoodPos -= .1;
    }
    else if (jStick.getPOV() == 180) {
      HoodPos += .1;
    }

    if (jStick.getRawButton(Constants.INPUT.attackMode)) {
      AttackMode();
    }
    else if (jStick.getRawButton(Constants.INPUT.defenceMode)) {
      DefenceMode();
    }

    if (jStick.getRawButton(Constants.INPUT.elevatorUp)) {
      Elevator.getInstance().setElevatorOutput(.420, 0);
      ElevatorStop.getInstance().setStopper(true);
    }
    else if (jStick.getRawButton(Constants.INPUT.elevatorDown)) {
      Elevator.getInstance().setElevatorOutput(-.420, 0);
      ElevatorStop.getInstance().setStopper(true);
    }
    else if(!isInAttackMode){
      Elevator.getInstance().setElevatorOutput(0, 0);
    }
  

    Shooter.getInstance().setHoodPos(HoodPos);
    // System.out.println("RPM: " + Shooter.getInstance().getShooterRPM() + " Hood
    // Pos: " + Shooter.getInstance().getHoodPos());

  }

  public void AttackMode() {
    Intake.getInstance().setIntakeRoller(.75);
    Intake.getInstance().setIntakeState(true);
    Elevator.getInstance().setElevatorOutput(.420, .420);
    ElevatorStop.getInstance().setStopper(false);

    isInAttackMode = true;
  }

  public void DefenceMode() {
    Intake.getInstance().setIntakeRoller(0);
    Intake.getInstance().setIntakeState(false);
    Elevator.getInstance().setElevatorOutput(0, 0);
    isInAttackMode = false;
  }

  public void Shoot(double TargetRPM, double HoodPos) {
    Shooter.getInstance().setShooterOutput(TargetRPM);
    Shooter.getInstance().setHoodPos(HoodPos);
    Shooter.getInstance().setShooterOutput(ShooterRPM);
    System.out.println("RPM: " + Shooter.getInstance().getShooterRPM() + " Hood Pos: " + Shooter.getInstance().getHoodPos());

    if (Shooter.getInstance().getShooterRPM() > TargetRPM) {
      Elevator.getInstance().setElevatorOutput(.75, .5);
      ElevatorStop.getInstance().setStopper(false);
    }
    else {
      Elevator.getInstance().setElevatorOutput(.25, .5);
    }
  }
}