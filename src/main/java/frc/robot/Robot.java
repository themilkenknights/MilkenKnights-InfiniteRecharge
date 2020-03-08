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

  private Joystick mDriverJoystick = new Joystick(0);
  private Joystick mOperatorJoystick = new Joystick(1);
  private Compressor mCompressor = new Compressor(0);

  private double mManualHoodPos = -0.1;
  private double mManualShooterSpeed = 2600;
  private boolean mIsInAttackMode;
  private Timer brakeTimer = new Timer();

  private Command m_autonomousCommand;
  private SendableChooser<AutoPosition> positionChooser = new SendableChooser<>();
  private ShuffleboardTab mTab = Shuffleboard.getTab("Match");
  private ComplexWidget positionChooserTab = mTab.add("Auto Chooser", positionChooser).withWidget(BuiltInWidgets.kSplitButtonChooser);

  private Drive mDrive = Drive.getInstance();
  private Shooter mShooter = Shooter.getInstance();
  private Elevator mElevator = Elevator.getInstance();
  private Limelight mLimelight = Limelight.getInstance();
  private ElevatorStopper mElevatorStopper = ElevatorStopper.getInstance();
  private Climber mClimber = Climber.getInstance();
  private Intake mIntake = Intake.getInstance();

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
    Shuffle.getInstance().update();
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
        //TODO: This may break things. Test this.
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
    defenseMode();
    mDrive.configBrakeMode();
    mDrive.zeroSensors();
  }

  @Override
  public void teleopPeriodic() {
    updateSensors();
    input();
  }

  public void input() {
    if (mDriverJoystick.getRawButton(Constants.INPUT.limeLight)) {
      mLimelight.autoAimShoot(false);
    } else if (mOperatorJoystick.getRawButton(1)) {
      mLimelight.autoAimShoot(true);
    } else {
      double forward, turn, rightOut, leftOut;
      forward = (-mDriverJoystick.getRawAxis(2) + mDriverJoystick.getRawAxis(3) + mDrive.antiTip());
      //TODO: LOOK BELOW AARON. CHHESY DRIVE ALREADY DOES THIS.
      turn = 0.9 * Math.pow(-mDriverJoystick.getRawAxis(0), 3); //TODO: VERY BAD. WE ARE ALREADY CUBING THE INPUTS. THIS IS DOING IT TWICE
      //TODO: LOOK ABOVE AARON
      DriveSignal controlSig = MkUtil.cheesyDrive(forward, turn, true);
      leftOut = controlSig.getLeft();
      rightOut = controlSig.getRight();

      if (mOperatorJoystick.getRawButtonPressed(Constants.INPUT.attackMode)) {
        mIsInAttackMode = true;
      } else if (mOperatorJoystick.getRawButtonPressed(Constants.INPUT.defenseMode)) {
        mIsInAttackMode = false;
      }

      if (mIsInAttackMode) {
        mDrive.setOutput(new DriveSignal(leftOut / 2.7, rightOut / 2.7));
        attackMode();
      } else {
        mDrive.setOutput(new DriveSignal(leftOut, rightOut));
        defenseMode();
      }

      if (mOperatorJoystick.getRawButtonPressed(Constants.INPUT.climbOn)) {
        mClimber.setClimbState(ClimbState.CLIMB);
      } else if (mOperatorJoystick.getRawButtonPressed(Constants.INPUT.climbOff)) {
        mClimber.setClimbState(ClimbState.RETRACT);
      }

      if (mOperatorJoystick.getButtonCount() > 0) {
        if (mOperatorJoystick.getPOV() == 0) {
          mManualHoodPos -= .05;
        } else if (mOperatorJoystick.getPOV() == 180) {
          mManualHoodPos += .05;
        }
      }

      if (mOperatorJoystick.getRawButtonPressed(6)) {
        mManualShooterSpeed += 100;
      } else if (mOperatorJoystick.getRawButtonPressed(4)) {
        mManualShooterSpeed -= 100;
      }

      if (mOperatorJoystick.getRawButton(3)) {
        mShooter.setHoodPos(MkUtil.limit(mManualHoodPos, -3.25, 0));
        mShooter.setShooterRPM(mManualShooterSpeed);
      } else {
        mShooter.setHoodPos(0);
        mShooter.setShooterOutput(0);
      }

      if (mOperatorJoystick.getRawButtonPressed(2)) {
        mElevatorStopper.setStopper(StopperState.GO);
      } else {
        mElevatorStopper.setStopper(StopperState.STOP);
      }

      if (mOperatorJoystick.getRawButton(Constants.INPUT.elevatorUp)) {
        mElevator.setElevatorOutput(.420);
      } else if (mOperatorJoystick.getRawButton(Constants.INPUT.elevatorDown)) {
        mElevator.setElevatorOutput(-.420);
      } else if (!mIsInAttackMode) {
        mElevator.setElevatorOutput(0);
      }

      if (mDriverJoystick.getRawButtonPressed(5)) {
        mLimelight.toggleLED();
      }
    }
  }

  public void attackMode() {
    mIntake.setIntakeRoller(.75);
    mIntake.setIntakeState(IntakeState.INTAKE);
    mElevator.setElevatorOutput(0.25);
    mElevatorStopper.setStopper(StopperState.STOP);
  }

  public void defenseMode() {
    mIntake.setIntakeRoller(0.0);
    mIntake.setIntakeState(IntakeState.STOW);
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
    mClimber.setClimbState(ClimbState.RETRACT);
    mIntake.setIntakeState(IntakeState.STOW);
    mElevatorStopper.setStopper(StopperState.STOP);
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
}
