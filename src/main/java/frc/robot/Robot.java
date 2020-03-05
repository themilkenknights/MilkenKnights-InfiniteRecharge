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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Climber.ClimbState;
import frc.robot.ElevatorStopper.StopperState;
import frc.robot.Intake.IntakeState;
import frc.robot.commands.Autonomous;
import frc.robot.lib.MkUtil;
import frc.robot.lib.MkUtil.DriveSignal;

public class Robot extends TimedRobot {

  private Joystick stick = new Joystick(0);
  private Joystick jStick = new Joystick(1);
  private Compressor mCompressor = new Compressor(0);
  private double hoodPos, shooterRPM, shooterSpeed;
  private boolean isInAttackMode, updateDashboard;
  private Command m_autonomousCommand;
  private Timer shootTimer = new Timer();
  private boolean shootOn = false;
  private double limeOffset = 0;

  public Robot() {
    super(Constants.kDt);
  }

  @Override
  public void robotInit() {
    mCompressor.start();
    Shooter.getInstance().zeroHood();
    m_autonomousCommand = new Autonomous();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Shuffle.getInstance().updateT();
    if (updateDashboard) {
      Shuffle.getInstance().update();
      updateDashboard = false;
    } else {
      updateDashboard = true;
    }
  }

  @Override
  public void autonomousInit() {
    Drive.getInstance().configBrakeMode();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    updateSensors();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    Drive.getInstance().configBrakeMode();
    Drive.getInstance().zeroSensors();
    hoodPos = shooterRPM = shooterSpeed = 0;
  }

  @Override
  public void teleopPeriodic() {
    updateSensors();
    input();
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void disabledInit() {
    Drive.getInstance().configCoastMode();
  }

  @Override
  public void disabledPeriodic() {
    updateSensors();
  }

  public void input() {
    if (jStick.getRawButton(Constants.INPUT.limeLight)) {
      Limelight.getInstance().autoAimShoot(limeOffset);
    } else {
      //Resets Turn Controller Integrator
      Limelight.getInstance().resetInt();

      double forward, turn, rightOut, leftOut;
      forward = (-stick.getRawAxis(2) + stick.getRawAxis(3) /*+ Drive.getInstance().antiTip()*/);
      turn = (-stick.getRawAxis(0));
      DriveSignal controlSig = MkUtil.cheesyDrive(forward, turn, true);
      leftOut = controlSig.getLeft();
      rightOut = controlSig.getRight();

      if (jStick.getRawButtonPressed(Constants.INPUT.attackMode)) {
        isInAttackMode = true;
      } else if (jStick.getRawButtonPressed(Constants.INPUT.defenseMode)) {
        isInAttackMode = false;
      }

      if (isInAttackMode) {
        Drive.getInstance().setOutput(new DriveSignal(leftOut / 2.0, rightOut / 2.0));
        AttackMode();
      } else {
        Drive.getInstance().setOutput(new DriveSignal(leftOut, rightOut));
        DefenceMode();
      }

      if (jStick.getRawButtonPressed(Constants.INPUT.climbOn)) {
        Climber.getInstance().setClimbState(ClimbState.CLIMB);
      } else if (jStick.getRawButtonPressed(Constants.INPUT.climbOff)) {
        Climber.getInstance().setClimbState(ClimbState.RETRACT);
      }

      if (jStick.getPOV() == 0) {
        hoodPos -= .05;
      } else if (jStick.getPOV() == 180) {
        hoodPos += .05;
      }

      if (jStick.getRawButtonPressed(6)) {
        shooterSpeed += .05;
      } else if (jStick.getRawButtonPressed(4)) {
        shooterSpeed -= .05;
      }

      if (jStick.getRawButtonPressed(10)) {
        ElevatorStopper.getInstance().toggleStopper();
      }

      if (jStick.getRawButtonPressed(1)) {
        Shooter.getInstance().setHoodPos(hoodPos);
        Shooter.getInstance().setShooterOutput(shooterSpeed);
      } else {
        Shooter.getInstance().setHoodPos(hoodPos);
        Shooter.getInstance().setShooterOutput(shooterSpeed);
      }

      if (jStick.getRawButton(Constants.INPUT.elevatorUp)) {
        Elevator.getInstance().setElevatorOutput(.420);
        ElevatorStopper.getInstance().setStopper(StopperState.GO);
      } else if (jStick.getRawButton(Constants.INPUT.elevatorDown)) {
        Elevator.getInstance().setElevatorOutput(-.420);
        ElevatorStopper.getInstance().setStopper(StopperState.GO);
      } else if (!isInAttackMode) {
        Elevator.getInstance().setElevatorOutput(0);
      }
    }
  }

  public void AttackMode() {
    Intake.getInstance().setIntakeRoller(.75);
    Intake.getInstance().setIntakeState(IntakeState.INTAKE);
    Elevator.getInstance().setElevatorOutput(0.35);
    Intake.getInstance().setHopperRoller(.42);
    ElevatorStopper.getInstance().setStopper(StopperState.STOP);
    isInAttackMode = true;
  }

  public void DefenceMode() {
    Intake.getInstance().setIntakeRoller(0.0);
    Intake.getInstance().setIntakeState(IntakeState.STOW);
    Elevator.getInstance().setElevatorOutput(0);
    isInAttackMode = false;
  }

  public void updateSensors() {
    Drive.getInstance().updateSensors();
    Limelight.getInstance().updateSensors();
  }
}
