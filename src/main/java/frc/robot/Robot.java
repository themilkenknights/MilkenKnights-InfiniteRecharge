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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Climber.ClimbState;
import frc.robot.ElevatorStopper.StopperState;
import frc.robot.Intake.IntakeState;
import frc.robot.commands.CenterAuto;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.LeftTrenchAuto;
import frc.robot.commands.RightAuto;
import frc.robot.lib.MkUtil;
import frc.robot.lib.MkUtil.DriveSignal;

public class Robot extends TimedRobot {

  private Joystick stick = new Joystick(0);
  private Joystick jStick = new Joystick(1);
  private Compressor mCompressor = new Compressor(0);

  private double hoodPos = -0.1;
  private double shooterSpeed = 2600;
  private boolean isInAttackMode, updateDashboard;
  private Timer brakeTimer = new Timer();

  private Command m_autonomousCommand;
  private SendableChooser<AutoPosition> positionChooser = new SendableChooser<>();
  private ShuffleboardTab mTab = Shuffleboard.getTab("Match");
  private ComplexWidget positionChooserTab = mTab.add("Auto Chooser", positionChooser).withWidget(BuiltInWidgets.kSplitButtonChooser);

  private Drive mDrive = Drive.getInstance();
  private Shooter mShooter = Shooter.getInstance();
  private Elevator mElevator = Elevator.getInstance();
  private Limelight mLimelight = Limelight.getInstance();

  public Robot() {
    super(Constants.kDt);
  }

  @Override
  public void robotInit() {
    Shuffleboard.startRecording();
    Shuffleboard.selectTab("Match");
    mCompressor.start();
    mShooter.zeroHood();
    positionChooser.addOption("Center", AutoPosition.CENTER);
    positionChooser.addOption("Nothing", AutoPosition.NOTHING);
    positionChooser.setDefaultOption("Left Trench", AutoPosition.LEFT);
    positionChooser.addOption("Right", AutoPosition.RIGHT);
    positionChooser.addOption("Drive Straight", AutoPosition.DRIVE_STRAIGHT);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Shuffle.getInstance().updateDeltaTime();
    if (updateDashboard) {
      Shuffle.getInstance().update();
      updateDashboard = false;
    } else {
      updateDashboard = true;
    }
  }

  @Override
  public void autonomousInit() {
    Shuffleboard.addEventMarker("Auto Init", EventImportance.kNormal);
    mDrive.zeroSensors();
    mDrive.configBrakeMode();
    switch (positionChooser.getSelected()) {
      case LEFT:
        m_autonomousCommand = new LeftTrenchAuto();
        break;
      case CENTER:
        m_autonomousCommand = new CenterAuto();
        break;
      case RIGHT:
        m_autonomousCommand = new RightAuto();
        break;
      case DRIVE_STRAIGHT:
        m_autonomousCommand = new DriveStraight(60);
        break;
      case NOTHING:
        // This may break things. Test this.
        break;
    }
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
    Shuffleboard.addEventMarker("Teleop Init", EventImportance.kNormal);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    DefenceMode();
    mDrive.configBrakeMode();
    mDrive.zeroSensors();
  }

  @Override
  public void teleopPeriodic() {
    updateSensors();
    input();
  }

  public void input() {
    if (stick.getRawButton(Constants.INPUT.limeLight)) {
      mLimelight.autoAimShoot(false);
    } else if (jStick.getRawButton(1)) {
      mLimelight.autoAimShoot(true);
    } else {
      ElevatorStopper.getInstance().setStopper(ElevatorStopper.StopperState.STOP);
      double forward, turn, rightOut, leftOut;
      forward = (-stick.getRawAxis(2) + stick.getRawAxis(3) + mDrive.antiTip());
      turn = 0.9 * Math.pow(-stick.getRawAxis(0), 3);
      DriveSignal controlSig = MkUtil.cheesyDrive(forward, turn, true);
      leftOut = controlSig.getLeft();
      rightOut = controlSig.getRight();

      if (jStick.getRawButtonPressed(Constants.INPUT.attackMode)) {
        isInAttackMode = true;
      } else if (jStick.getRawButtonPressed(Constants.INPUT.defenseMode)) {
        isInAttackMode = false;
      }

      if (isInAttackMode) {
        mDrive.setOutput(new DriveSignal(leftOut / 2.7, rightOut / 2.7));
        AttackMode();
      } else {
        mDrive.setOutput(new DriveSignal(leftOut, rightOut));
        DefenceMode();
      }

      if (jStick.getRawButtonPressed(Constants.INPUT.climbOn)) {
        Climber.getInstance().setClimbState(ClimbState.CLIMB);
      } else if (jStick.getRawButtonPressed(Constants.INPUT.climbOff)) {
        Climber.getInstance().setClimbState(ClimbState.RETRACT);
      }

      if (jStick.getButtonCount() > 0) {
        if (jStick.getPOV() == 0) {
          hoodPos -= .05;
        } else if (jStick.getPOV() == 180) {
          hoodPos += .05;
        }
      }

      if (jStick.getRawButtonPressed(6)) {
        shooterSpeed += 100;
      } else if (jStick.getRawButtonPressed(4)) {
        shooterSpeed -= 100;
      }

      if (jStick.getRawButton(3)) {
        mShooter.setHoodPos(limit(hoodPos, -3.25, 0));
        mShooter.setShooterRPM(shooterSpeed);
      } else {
        mShooter.setHoodPos(0);
        mShooter.setShooterOutput(0);
      }

      if (jStick.getRawButtonPressed(2)) {
        ElevatorStopper.getInstance().setStopper(StopperState.GO);
      } else {
        ElevatorStopper.getInstance().setStopper(StopperState.STOP);
      }

      if (jStick.getRawButton(Constants.INPUT.elevatorUp)) {
        mElevator.setElevatorOutput(.420);
      } else if (jStick.getRawButton(Constants.INPUT.elevatorDown)) {
        mElevator.setElevatorOutput(-.420);
      } else if (!isInAttackMode) {
        mElevator.setElevatorOutput(0);
      }

      if (stick.getRawButtonPressed(5)) {
        mLimelight.toggleLED();
      }
    }
  }

  public void AttackMode() {
    Intake.getInstance().setIntakeRoller(.75);
    Intake.getInstance().setIntakeState(IntakeState.INTAKE);
    mElevator.setElevatorOutput(0.25);
    ElevatorStopper.getInstance().setStopper(StopperState.STOP);
  }

  public void DefenceMode() {
    Intake.getInstance().setIntakeRoller(0.0);
    Intake.getInstance().setIntakeState(IntakeState.STOW);
    mElevator.setElevatorOutput(0);
  }

  public void updateSensors() {
    mDrive.updateSensors();
    mLimelight.updateSensors();
  }

  @Override
  public void disabledInit() {
    Shuffleboard.addEventMarker("Disabled Init", EventImportance.kNormal);
    brakeTimer.reset();
    brakeTimer.start();
    Climber.getInstance().setClimbState(ClimbState.RETRACT); // When disabled reset to default state for safety
    Intake.getInstance().setIntakeState(IntakeState.STOW);
    ElevatorStopper.getInstance().setStopper(StopperState.STOP);
  }

  @Override
  public void disabledPeriodic() {
    updateSensors();
    if (brakeTimer.hasElapsed(1.5)) {
      mDrive.configCoastMode();
    }
  }

  public enum AutoPosition {
    LEFT, NOTHING, RIGHT, CENTER, DRIVE_STRAIGHT
  }

  public double limit(double value, double min, double max) {
    if (value > max) {
      return max;
    } else if (value < min) {
      return min;
    } else {
      return value;
    }
  }
}
