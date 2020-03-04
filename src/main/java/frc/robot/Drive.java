package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DRIVE;
import frc.robot.lib.MkUtil;
import frc.robot.lib.MkUtil.DriveSignal;

public class Drive {

  private final AHRS navX = new AHRS();
  private final TalonFX leftMaster = new TalonFX(CAN.kDriveLeftMasterId);
  private final TalonFX leftSlave = new TalonFX(CAN.kDriveLeftSlaveId);
  private final TalonFX rightMaster = new TalonFX(CAN.kDriveRightMasterId);
  private final TalonFX rightSlave = new TalonFX(CAN.kDriveRightSlaveId);
  private double rollOffset, lastVel, lastTime, magicTarget, magicStraightTarget;
  private boolean isOnMagicTarget = false;

  private Drive() {
    leftMaster.configFactoryDefault();
    leftSlave.configFactoryDefault();
    rightMaster.configFactoryDefault();
    rightSlave.configFactoryDefault();

    leftMaster.configVoltageCompSaturation(12.0);
    leftMaster.enableVoltageCompensation(true);
    leftSlave.configVoltageCompSaturation(12.0);
    leftSlave.enableVoltageCompensation(true);
    rightMaster.configVoltageCompSaturation(12.0);
    rightMaster.enableVoltageCompensation(true);
    rightSlave.configVoltageCompSaturation(12.0);
    rightSlave.enableVoltageCompensation(true);

    leftMaster.setInverted(DRIVE.kLeftMasterInverted);
    leftSlave.setInverted(DRIVE.kLeftSlaveInverted);
    rightMaster.setInverted(DRIVE.kRightMasterInverted);
    rightSlave.setInverted(DRIVE.kRightSlaveInverted);

    leftMaster.setSensorPhase(DRIVE.kLeftSensorInverted);
    rightMaster.setSensorPhase(DRIVE.kRightSensorInverted);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    leftMaster.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);

    leftSlave.setNeutralMode(NeutralMode.Brake);
    rightSlave.setNeutralMode(NeutralMode.Brake);

    leftMaster.config_kF(0, DRIVE.kDriveKf);
    leftMaster.config_kP(0, DRIVE.kDriveKp);
    leftMaster.config_kI(0, 0);
    leftMaster.config_kD(0, DRIVE.kDriveKD);

    rightMaster.config_kF(0, DRIVE.kDriveKf);
    rightMaster.config_kP(0, DRIVE.kDriveKp);
    rightMaster.config_kI(0, 0);
    rightMaster.config_kD(0, DRIVE.kDriveKD);

    /* Set acceleration and vcruise velocity - see documentation */
    leftMaster.configMotionCruiseVelocity((int) (0.08 * DRIVE.kMaxNativeVel));
    leftMaster.configMotionAcceleration((int) (0.05 * DRIVE.kMaxNativeVel));
    leftMaster.configAllowableClosedloopError(0, 20);
    leftMaster.configNeutralDeadband(0.001);
    leftMaster.setStatusFramePeriod(StatusFrame.Status_1_General, 5);
    leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5);
    leftMaster.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10);
    leftMaster.configClosedLoopPeakOutput(0, 1.0);
    rightMaster.configSelectedFeedbackCoefficient(1.0 / 10.75);
    leftMaster.configMotionSCurveStrength(7);

    rightMaster.configMotionCruiseVelocity((int) (0.08 * DRIVE.kMaxNativeVel));
    rightMaster.configMotionAcceleration((int) (0.05 * DRIVE.kMaxNativeVel));
    rightMaster.configAllowableClosedloopError(0, 20);
    rightMaster.configNeutralDeadband(0.001);
    rightMaster.setStatusFramePeriod(StatusFrame.Status_1_General, 5);
    rightMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5);
    rightMaster.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10);
    rightMaster.configClosedLoopPeakOutput(0, 1.0);
    leftMaster.configSelectedFeedbackCoefficient(1.0 / 10.75);
    rightMaster.configMotionSCurveStrength(7);

    /* Zero the sensor once on robot boot up */
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  public static Drive getInstance() {
    return InstanceHolder.mInstance;
  }

  public void setDriveStraight(double dist) {
    magicStraightTarget = dist;
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  public void updateDriveStraight() {
    leftMaster.set(ControlMode.MotionMagic, MkUtil.inToNative(magicStraightTarget));
    rightMaster.set(ControlMode.MotionMagic, MkUtil.inToNative(magicStraightTarget));
  }

  public boolean isDriveStraightDone() {
    double err =
        magicStraightTarget - ((MkUtil.nativeToIn(leftMaster.getSelectedSensorPosition()) + MkUtil.nativeToIn(rightMaster.getSelectedSensorPosition())) / 2.0);
    if (Math.abs(err) < 0.75 && MkUtil.nativeToInPer100Ms(getVelocity()) < 0.1) {
      leftMaster.set(ControlMode.PercentOutput, 0);
      rightMaster.set(ControlMode.PercentOutput, 0);
      return true;
    }
    return false;
  }

  public void setMagicTurnInPlace(double deg) {
    navX.zeroYaw();
    isOnMagicTarget = false;
    magicTarget = getYaw() + deg;
  }

  public void magicTurnInPlaceUpdate() {
    double error_deg = getYaw() - magicTarget;
    double error_rad = Math.toRadians(error_deg);
    double delta_v = 22.97 * error_rad / (2 * 0.95);
    leftMaster.set(ControlMode.MotionMagic, MkUtil.inToNative(-delta_v) + leftMaster.getSelectedSensorPosition());
    rightMaster.set(ControlMode.MotionMagic, MkUtil.inToNative(delta_v) + rightMaster.getSelectedSensorPosition());
    if (Math.abs(error_deg) < 4.0 && Math.abs(MkUtil.nativeToInPer100Ms(leftMaster.getSelectedSensorVelocity())) < 0.25
        && Math.abs(MkUtil.nativeToInPer100Ms(rightMaster.getSelectedSensorVelocity())) < 0.25) {
      isOnMagicTarget = true;
    }
    SmartDashboard.putNumber("Error Deg", error_deg);
    SmartDashboard.putNumber("Delta V", delta_v);
    SmartDashboard.putBoolean("Magic Turn Done", isOnMagicTarget);
  }

  public boolean isMagicOnTarget() {
    if (isOnMagicTarget) {
      leftMaster.set(ControlMode.PercentOutput, 0);
      rightMaster.set(ControlMode.PercentOutput, 0);
      return true;
    }
    return false;
  }

  public void setOutput(DriveSignal signal) {
    leftMaster.set(ControlMode.PercentOutput, signal.getLeft());
    rightMaster.set(ControlMode.PercentOutput, signal.getRight());
  }

  public double antiTip() {
    double roll = getRoll();
    SmartDashboard.putNumber("NavX Roll", roll);

    // Simplified Logic Below
    if (Math.abs(roll) >= DRIVE.kAntiTipThreshold) {
      return Math.sin(Math.toRadians(roll)) * -2;
    } else {
      return 0.0;
    }
  }

  public void configCoastMode() {
    leftMaster.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);
    leftSlave.setNeutralMode(NeutralMode.Brake);
    rightSlave.setNeutralMode(NeutralMode.Brake);
  }

  public void configBrakeMode() {
    leftMaster.setNeutralMode(NeutralMode.Coast);
    rightMaster.setNeutralMode(NeutralMode.Coast);
    leftSlave.setNeutralMode(NeutralMode.Coast);
    rightSlave.setNeutralMode(NeutralMode.Coast);
  }

  public double getPos() {
    return rightMaster.getSelectedSensorPosition();
  }

  public double getVelocity() {
    return (rightMaster.getSelectedSensorVelocity() + leftMaster.getSelectedSensorVelocity()) / 2.0;
  }

  public double getAcceleration() {
    double curVel = rightMaster.getSelectedSensorVelocity();
    double curTime = Timer.getFPGATimestamp();
    double acc = (curVel - lastVel) / (Timer.getFPGATimestamp() - lastTime);
    lastVel = curVel;
    lastTime = curTime;
    return acc;
  }

  public double getRoll() {
    return navX.getRoll() - rollOffset;
  }

  public void resetNavX() {
    rollOffset = navX.getRoll();
    navX.zeroYaw();
  }

  public double getYaw() {
    return navX.getAngle();
  }

  private static class InstanceHolder {

    private static final Drive mInstance = new Drive();
  }
}
