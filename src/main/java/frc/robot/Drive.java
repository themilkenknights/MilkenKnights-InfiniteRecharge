package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
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
  private double magicTarget, magicStraightTarget;
  private PeriodicIO mPeriodicIO;

  private Drive() {
    mPeriodicIO = new PeriodicIO();
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
    leftMaster.config_kI(0, DRIVE.kDriveKi);
    leftMaster.config_kD(0, DRIVE.kDriveKD);

    rightMaster.config_kF(0, DRIVE.kDriveKf);
    rightMaster.config_kP(0, DRIVE.kDriveKp);
    rightMaster.config_kI(0, DRIVE.kDriveKi);
    rightMaster.config_kD(0, DRIVE.kDriveKD);

    /* Set acceleration and vcruise velocity - see documentation */
    leftMaster.configMotionCruiseVelocity(DRIVE.kMotionMagicStraightVel);
    leftMaster.configMotionAcceleration(DRIVE.kMotionMagicStraightAccel);
    leftMaster.configAllowableClosedloopError(0, 1);
    leftMaster.configNeutralDeadband(0.0001);
    leftMaster.setStatusFramePeriod(StatusFrame.Status_1_General, 5);
    leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5);
    leftMaster.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10);
    leftMaster.configClosedLoopPeakOutput(0, 1.0);
    rightMaster.configSelectedFeedbackCoefficient(1.0 / 10.75);
    leftMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_25Ms);
    leftMaster.configVelocityMeasurementWindow(16);
    leftMaster.configMotionSCurveStrength(6);

    rightMaster.configMotionCruiseVelocity(DRIVE.kMotionMagicStraightVel);
    rightMaster.configMotionAcceleration(DRIVE.kMotionMagicStraightAccel);
    rightMaster.configAllowableClosedloopError(0, 1);
    rightMaster.configNeutralDeadband(0.0001);
    rightMaster.setStatusFramePeriod(StatusFrame.Status_1_General, 5);
    rightMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5);
    rightMaster.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10);
    rightMaster.configClosedLoopPeakOutput(0, 1.0);
    leftMaster.configSelectedFeedbackCoefficient(1.0 / 10.75);
    rightMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_25Ms);
    rightMaster.configVelocityMeasurementWindow(16);
    rightMaster.configMotionSCurveStrength(6);

    zeroSensors();
  }

  public static Drive getInstance() {
    return InstanceHolder.mInstance;
  }

  public void updateSensors() {
    mPeriodicIO.timestamp = Timer.getFPGATimestamp();

    mPeriodicIO.yaw_continouous = getYaw();
    mPeriodicIO.roll = navX.getRoll();

    mPeriodicIO.left_vel_native = leftMaster.getSelectedSensorVelocity();
    mPeriodicIO.right_vel_native = leftMaster.getSelectedSensorVelocity();
    mPeriodicIO.left_pos_native = leftMaster.getSelectedSensorPosition();
    mPeriodicIO.right_pos_native = leftMaster.getSelectedSensorPosition();

    mPeriodicIO.left_vel_inches_per_sec = MkUtil.nativePer100MstoInchesPerSec(mPeriodicIO.left_vel_native);
    mPeriodicIO.right_vel_inches_per_sec = MkUtil.nativePer100MstoInchesPerSec(mPeriodicIO.right_vel_native);
    mPeriodicIO.left_pos_inches = MkUtil.nativeToInches(mPeriodicIO.left_pos_native);
    mPeriodicIO.right_pos_inches = MkUtil.nativeToInches(mPeriodicIO.right_pos_native);

    mPeriodicIO.left_vel_meters_per_sec = MkUtil.nativePer100MsToMetersPerSec(mPeriodicIO.left_vel_native);
    mPeriodicIO.right_vel_meters_per_sec = MkUtil.nativePer100MsToMetersPerSec(mPeriodicIO.right_vel_native);
    mPeriodicIO.left_pos_meters = MkUtil.nativeToMeters(mPeriodicIO.left_pos_native);
    mPeriodicIO.right_pos_meters = MkUtil.nativeToInches(mPeriodicIO.right_pos_native);

    mPeriodicIO.avg_dist_inches = (mPeriodicIO.left_pos_inches + mPeriodicIO.right_pos_inches) / 2.0;
    mPeriodicIO.avg_vel_inches_per_sec = (mPeriodicIO.left_vel_inches_per_sec + mPeriodicIO.right_vel_inches_per_sec) / 2.0;
  }

  public void setDriveStraight(double dist) {
    leftMaster.configMotionCruiseVelocity(DRIVE.kMotionMagicStraightVel);
    leftMaster.configMotionAcceleration(DRIVE.kMotionMagicStraightAccel);
    rightMaster.configMotionCruiseVelocity(DRIVE.kMotionMagicStraightVel);
    rightMaster.configMotionAcceleration(DRIVE.kMotionMagicStraightAccel);
    magicStraightTarget = dist;
    zeroSensors();
  }

  public void updateDriveStraight() {
    leftMaster.set(ControlMode.MotionMagic, MkUtil.inchesToNative(magicStraightTarget));
    rightMaster.set(ControlMode.MotionMagic, MkUtil.inchesToNative(magicStraightTarget));
    mPeriodicIO.left_output = MkUtil.inchesToNative(magicStraightTarget);
    mPeriodicIO.right_output = MkUtil.inchesToNative(magicStraightTarget);
  }

  public boolean isDriveStraightDone() { //No Longer Sets Output to Zero when finished
    double err = magicStraightTarget - mPeriodicIO.avg_dist_inches;
    return Math.abs(err) < 0.5 && Math.abs(mPeriodicIO.avg_vel_inches_per_sec) < 0.1;
  }

  public void setMagicTurnInPlace(double deg) {
    leftMaster.configMotionCruiseVelocity(DRIVE.kMotionMagicTurnInPlaceVel);
    leftMaster.configMotionAcceleration(DRIVE.kMotionMagicTurnInPlaceAccel);
    rightMaster.configMotionCruiseVelocity(DRIVE.kMotionMagicTurnInPlaceVel);
    rightMaster.configMotionAcceleration(DRIVE.kMotionMagicTurnInPlaceAccel);
    magicTarget = navX.getAngle() + deg;
  }

  public void magicTurnInPlaceUpdate() {
    double error_deg = mPeriodicIO.yaw_continouous - magicTarget;
    double error_rad = Math.toRadians(error_deg);//22.97
    double delta_v = 22.97 * error_rad / (2 * 0.95);

    double left_out = MkUtil.inchesToNative(-delta_v) + leftMaster.getSelectedSensorPosition();
    double right_out = MkUtil.inchesToNative(delta_v) + rightMaster.getSelectedSensorPosition();

    leftMaster.set(ControlMode.MotionMagic, left_out);
    rightMaster.set(ControlMode.MotionMagic, right_out);

    mPeriodicIO.left_output = left_out;
    mPeriodicIO.right_output = right_out;
  }

  public boolean isMagicTurnInPlaceDone() { //No Longer Sets Output to Zero when finished
    return Math.abs(mPeriodicIO.yaw_continouous - magicTarget) < 3.0 && Math.abs(mPeriodicIO.avg_vel_inches_per_sec) < 0.1;
  }

  public void setOutput(DriveSignal signal) {
    leftMaster.set(ControlMode.PercentOutput, signal.getLeft());
    rightMaster.set(ControlMode.PercentOutput, signal.getRight());
    mPeriodicIO.left_output = signal.getLeft();
    mPeriodicIO.right_output = signal.getRight();
  }

  public double antiTip() {
    if (Math.abs(mPeriodicIO.roll) >= DRIVE.kAntiTipThreshold) {
      return Math.sin(Math.toRadians(navX.getRoll())) * -2;
    } else {
      return 0.0;
    }
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("Left Output", mPeriodicIO.left_output);
    SmartDashboard.putNumber("Right Output", mPeriodicIO.left_output);
    SmartDashboard.putNumber("Left Pos Inches", mPeriodicIO.left_pos_inches);
    SmartDashboard.putNumber("Right Pos Inches", mPeriodicIO.right_pos_inches);
    SmartDashboard.putNumber("Left Vel Inches/Sec", mPeriodicIO.left_vel_inches_per_sec);
    SmartDashboard.putNumber("Right Vel Inches/Sec", mPeriodicIO.right_vel_inches_per_sec);
    SmartDashboard.putNumber("NavX Roll", mPeriodicIO.roll);
    SmartDashboard.putNumber("NavX Yaw Continuous", mPeriodicIO.yaw_continouous);
    SmartDashboard.putNumber("Error Deg Turn In Place", mPeriodicIO.yaw_continouous - magicTarget);
    SmartDashboard.putNumber("Delta V Turn In Place", 22.97 * Math.toRadians(mPeriodicIO.yaw_continouous - magicTarget) / (2 * 0.95));
    SmartDashboard.putBoolean("Magic Turn In Place Done", isMagicTurnInPlaceDone());
  }

  public void configCoastMode() {
    leftMaster.setNeutralMode(NeutralMode.Coast);
    rightMaster.setNeutralMode(NeutralMode.Coast);
    leftSlave.setNeutralMode(NeutralMode.Coast);
    rightSlave.setNeutralMode(NeutralMode.Coast);
  }

  public void configBrakeMode() {
    leftMaster.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);
    leftSlave.setNeutralMode(NeutralMode.Brake);
    rightSlave.setNeutralMode(NeutralMode.Brake);
  }

  public void zeroSensors() {
    navX.zeroYaw();
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  public double getAvgVel() {
    return mPeriodicIO.avg_vel_inches_per_sec;
  }

  public double getYaw() {
    return navX.getAngle();
  }

  private static class InstanceHolder {

    private static final Drive mInstance = new Drive();
  }

  public static class PeriodicIO {
    public double timestamp;

    public double yaw_normalized;
    public double yaw_continouous;
    public double roll;

    public int left_vel_native;
    public int right_vel_native;
    public int left_pos_native;
    public int right_pos_native;

    public double left_vel_inches_per_sec;
    public double right_vel_inches_per_sec;
    public double left_pos_inches;
    public double right_pos_inches;

    public double left_vel_meters_per_sec;
    public double right_vel_meters_per_sec;
    public double left_pos_meters;
    public double right_pos_meters;

    public double avg_dist_inches;
    public double avg_vel_inches_per_sec;

    public double left_output;
    public double right_output;
  }
}
