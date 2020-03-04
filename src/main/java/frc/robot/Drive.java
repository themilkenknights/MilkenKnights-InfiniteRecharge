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

public class Drive {

  private AHRS navX = new AHRS();
  private TalonFX leftMaster = new TalonFX(CAN.driveLeftMasterId);
  private TalonFX leftSlave = new TalonFX(CAN.driveLeftSlaveId);
  private TalonFX rightMaster = new TalonFX(CAN.driveRightMasterId);
  private TalonFX rightSlave = new TalonFX(CAN.driveRightSlaveId);
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

    leftMaster.setInverted(DRIVE.leftMasterInverted);
    leftSlave.setInverted(DRIVE.leftSlaveInverted);
    rightMaster.setInverted(DRIVE.rightMasterInverted);
    rightSlave.setInverted(DRIVE.rightSlaveInverted);

    leftMaster.setSensorPhase(DRIVE.leftSensorInverted);
    rightMaster.setSensorPhase(DRIVE.rightSensorInverted);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    leftMaster.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);

    leftSlave.setNeutralMode(NeutralMode.Brake);
    rightSlave.setNeutralMode(NeutralMode.Brake);

    leftMaster.config_kF(0, DRIVE.kF);
    leftMaster.config_kP(0, DRIVE.kP);
    leftMaster.config_kI(0, 0);
    leftMaster.config_kD(0, DRIVE.kD);

    rightMaster.config_kF(0, DRIVE.kF);
    rightMaster.config_kP(0, DRIVE.kP);
    rightMaster.config_kI(0, 0);
    rightMaster.config_kD(0, DRIVE.kD);

    /* Set acceleration and vcruise velocity - see documentation */
    leftMaster.configMotionCruiseVelocity((int) (0.08 * DRIVE.kMaxNativeVel));
    leftMaster.configMotionAcceleration((int) (0.05 * DRIVE.kMaxNativeVel));
    leftMaster.configAllowableClosedloopError(0, 20);
    leftMaster.configNeutralDeadband(0.001);
    leftMaster.setStatusFramePeriod(StatusFrame.Status_1_General, 5);
    leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5);
    leftMaster.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10);
    leftMaster.configClosedLoopPeakOutput(0, 1.0);
    rightMaster.configClosedLoopPeakOutput(0, 1.0);
    rightMaster.configMotionCruiseVelocity((int) (0.08 * DRIVE.kMaxNativeVel));
    rightMaster.configMotionAcceleration((int) (0.05 * DRIVE.kMaxNativeVel));
    rightMaster.configAllowableClosedloopError(0, 20);
    rightMaster.configNeutralDeadband(0.001);
    rightMaster.setStatusFramePeriod(StatusFrame.Status_1_General, 5);
    rightMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5);
    rightMaster.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10);
    leftMaster.configSelectedFeedbackCoefficient(1.0 / 10.75);
    rightMaster.configSelectedFeedbackCoefficient(1.0 / 10.75);
    leftMaster.configMotionSCurveStrength(7);
    rightMaster.configMotionSCurveStrength(7);

    /* Zero the sensor once on robot boot up */
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  public void setDriveStraight(double dist) {
    magicStraightTarget = dist;
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  public void updateDriveStraight() {
    leftMaster.set(ControlMode.MotionMagic, InchesToNativeUnits(magicStraightTarget));
    rightMaster.set(ControlMode.MotionMagic, InchesToNativeUnits(magicStraightTarget));
  }

  public boolean isDriveStraightDone() {
    double err = magicStraightTarget - ((nativeUnitsToInches(leftMaster.getSelectedSensorPosition())
        + nativeUnitsToInches(rightMaster.getSelectedSensorPosition())) / 2.0);
    if (Math.abs(err) < 0.75 && nativeUnitsPer100MstoInchesPerSec(getVelocity()) < 0.1) {
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
    leftMaster.set(ControlMode.MotionMagic, InchesToNativeUnits(-delta_v) + leftMaster.getSelectedSensorPosition());
    rightMaster.set(ControlMode.MotionMagic, InchesToNativeUnits(delta_v) + rightMaster.getSelectedSensorPosition());
    if (Math.abs(error_deg) < 4.0
        && Math.abs(nativeUnitsPer100MstoInchesPerSec(leftMaster.getSelectedSensorVelocity())) < 0.25
        && Math.abs(nativeUnitsPer100MstoInchesPerSec(rightMaster.getSelectedSensorVelocity())) < 0.25) {
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
    SmartDashboard.putNumber("Roll", roll);
    SmartDashboard.putNumber("Anti-Tip", Math.sin(Math.toRadians(roll)) * -2);
    // Simplified Logic Below
    if (Math.abs(roll) >= Constants.DRIVE.AngleThresholdDegrees) {
      return Math.sin(Math.toRadians(roll)) * -2;
    } else {
      return 0.0;
    }
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

  public double getYawVel() {
    return navX.getRate();
  }

  public static Drive getInstance() {
    return InstanceHolder.mInstance;
  }

  public double nativeUnitsToInches(double nativeUnits) {
    return (nativeUnits / 2048.0) * DRIVE.wheelCircumference;
  }

  private double InchesToNativeUnits(double inches) {
    return (inches / DRIVE.wheelCircumference) * 2048.0;
  }

  private double nativeUnitsPer100MstoInchesPerSec(double vel) {
    return 10 * nativeUnitsToInches(vel);
  }

  private double InchesPerSecToUnitsPer100Ms(double vel) {
    return InchesToNativeUnits(vel) / 10;
  }

  private static class InstanceHolder {

    private static final Drive mInstance = new Drive();
  }

  public static class DriveSignal {

    public static DriveSignal STOP = new DriveSignal(0, 0);
    protected double mLeftMotor;
    protected double mRightMotor;

    public DriveSignal(double left, double right) {
      mLeftMotor = left;
      mRightMotor = right;
    }

    public double getLeft() {
      return mLeftMotor;
    }

    public double getRight() {
      return mRightMotor;
    }

    @Override
    public String toString() {
      return "L: " + mLeftMotor + " R: " + mRightMotor;
    }
  }
}
