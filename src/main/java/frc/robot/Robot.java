/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.SlewRateLimiter;
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
import frc.robot.commands.BackAuto;
import frc.robot.commands.CenterAuto;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.LeftTrenchAuto;
import frc.robot.commands.RightAuto;
import frc.robot.lib.MkJoystick;
import frc.robot.lib.MkUtil;
import frc.robot.lib.MkUtil.DriveSignal;

public class Robot extends TimedRobot {

  private MkJoystick mDriverJoystick = new MkJoystick(0);
  private MkJoystick mOperatorJoystick = new MkJoystick(1);
  private Compressor mCompressor = new Compressor(0);

  private double mManualHoodPos = -0.1;
  private double mManualShooterSpeed = 2600;
  private boolean mIsInAttackMode, mAutoShooting, mDriverAssist;
  private Timer brakeTimer = new Timer();
  private Timer shootTimer = new Timer();

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

  private SlewRateLimiter accelLimiter = new SlewRateLimiter(Constants.DRIVE.kAccelLimit);

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
    positionChooser.addOption("Back Auto", AutoPosition.BACK);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Shuffle.getInstance().update();
  }

  @Override
  public void autonomousInit() {
    Shuffleboard.addEventMarker("Auto Init", EventImportance.kNormal);
    mDrive.resetOdometry();
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
      case BACK:
        m_autonomousCommand = new BackAuto();
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
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    Shuffleboard.addEventMarker("Teleop Init", EventImportance.kNormal);
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
    if (mDriverJoystick.getRawButton(1, "Full Limelight Shoot")) {
      mLimelight.autoAimShoot();
      shootTimer.start();
      mAutoShooting = true;
    } else {
      mAutoShooting = false;
      double forward = accelLimiter.calculate(-mDriverJoystick.getRawAxis(2) + mDriverJoystick.getRawAxis(3));
      double antiTip = mIsInAttackMode ? mDrive.antiTip() / 2.0 : mDrive.antiTip();
      double turn = -mDriverJoystick.getRawAxis(0);
      DriveSignal controlSig = MkUtil.cheesyDrive(forward, turn, true);
      double assist = mDriverAssist ? mLimelight.updateAutoAimOutput() : 0;
      mDrive.setOutput(controlSig.getLeftVel() + assist, controlSig.getRightVel() - assist, antiTip, antiTip);

      if (mOperatorJoystick.getRawButton(26, "Driver Assist Fire")) {
        mLimelight.spinUpShoot(5.0, Shooter.ShootingMode.VISION_ASSIST_SHOOTING);
        mIsInAttackMode = false;
      } else if (mDriverAssist) {
        mShooter.setShooterRPM(2000);
      }

      if (mDriverJoystick.getRawButtonPressed(7, "Driver Assist")) {
        mDriverAssist = !mDriverAssist;
      }

      if (mOperatorJoystick.getRawButtonPressed(Constants.INPUT.attackMode, "Attack Mode")) {
        mIsInAttackMode = true;
      } else if (mOperatorJoystick.getRawButtonPressed(Constants.INPUT.defenseMode, "Defense Mode")) {
        mIsInAttackMode = false;
      }

      if (mIsInAttackMode) {
        attackMode();
      } else {
        defenseMode();
      }

      if (mOperatorJoystick.getRawButtonPressed(Constants.INPUT.climbOn, "Extend Climb Actuators")) {
        mClimber.setClimbState(ClimbState.CLIMB);
      } else if (mOperatorJoystick.getRawButtonPressed(Constants.INPUT.climbOff, "Retract Climb Actuators")) {
        mClimber.setClimbState(ClimbState.RETRACT);
      }

      if (mOperatorJoystick.getButtonCount() > 0) {
        if (mOperatorJoystick.getPOV() == 0) {
          mManualShooterSpeed += 2;
        } else if (mOperatorJoystick.getPOV() == 180) {
          mManualShooterSpeed -= 2;
        }
      }

      if (mOperatorJoystick.getRawButton(1, "Set Manual RPM/Hood Pos")) {
        mShooter.setHoodPos(MkUtil.limit(mManualHoodPos, -3.25, 0));
        mShooter.setShooterRPM(mManualShooterSpeed);
        shootTimer.start();
        mShooter.setShootingMode(Shooter.ShootingMode.MANUAL_RPM);
      } else if (shootTimer.hasElapsed(0.25) && !mDriverAssist) {
        mShooter.setHoodPos(0);
        mShooter.setShooterOutput(0);
      }

      if (mOperatorJoystick.getRawButton(4, "Set Elevator Stopper To GO")) {
        mElevatorStopper.setStopper(ElevatorStopper.StopperState.GO);
      } else {
        mElevatorStopper.setStopper(ElevatorStopper.StopperState.STOP);
      }

      if (mDriverJoystick.getRawButtonPressed(5, "Toggle Limelight LED")) {
        mLimelight.toggleLED();
      }
    }

    if (mOperatorJoystick.getRawButton(Constants.INPUT.elevatorUp, "Set Elevator Up to 0.420")) {
      mElevator.setElevatorOutput(.420);
    } else if (mOperatorJoystick.getRawButton(Constants.INPUT.elevatorDown, "Set Elevator Down to -0.420")) {
      mElevator.setElevatorOutput(-.420);
    } else if (!mAutoShooting && mIsInAttackMode) {
      mElevator.setElevatorOutput(0.25);
    } else if (!mAutoShooting) {
      mElevator.setElevatorOutput(0);
    }
  }

  public void attackMode() {
    mIntake.setIntakeRoller(.75);
    mIntake.setIntakeState(IntakeState.INTAKE);
  }

  public void defenseMode() {
    mIntake.setIntakeRoller(0.0);
    mIntake.setIntakeState(IntakeState.STOW);
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
    LEFT, NOTHING, RIGHT, CENTER, DRIVE_STRAIGHT, BACK
  }
}
