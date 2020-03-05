package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  private double rollOffset, magicTarget, magicStraightTarget;
  private final DifferentialDriveOdometry m_odometry;
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
    leftMaster.config_IntegralZone(0, 100);
    leftMaster.configMaxIntegralAccumulator(0, 50);

    rightMaster.config_kF(0, DRIVE.kDriveKf);
    rightMaster.config_kP(0, DRIVE.kDriveKp);
    rightMaster.config_kI(0, DRIVE.kDriveKi);
    rightMaster.config_kD(0, DRIVE.kDriveKD);
    rightMaster.config_IntegralZone(0, 100);
    rightMaster.configMaxIntegralAccumulator(0, 50);



    /* Set acceleration and vcruise velocity - see documentation */
    leftMaster.configMotionCruiseVelocity((int) (0.25 * DRIVE.kMaxNativeVel));
    leftMaster.configMotionAcceleration((int) (0.13 * DRIVE.kMaxNativeVel));
    leftMaster.configAllowableClosedloopError(0, 1);
    leftMaster.configNeutralDeadband(0.0001);
    leftMaster.setStatusFramePeriod(StatusFrame.Status_1_General, 5);
    leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5);
    leftMaster.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10);
    leftMaster.configClosedLoopPeakOutput(0, 1.0);
    rightMaster.configSelectedFeedbackCoefficient(1.0 / 10.75);
    leftMaster.configMotionSCurveStrength(6);
    leftMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_25Ms);
    leftMaster.configVelocityMeasurementWindow(7);

    rightMaster.configMotionCruiseVelocity((int) (0.25 * DRIVE.kMaxNativeVel));
    rightMaster.configMotionAcceleration((int) (0.13 * DRIVE.kMaxNativeVel));
    rightMaster.configAllowableClosedloopError(0, 1);
    rightMaster.configNeutralDeadband(0.0001);
    rightMaster.setStatusFramePeriod(StatusFrame.Status_1_General, 5);
    rightMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5);
    rightMaster.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic, 10);
    rightMaster.configClosedLoopPeakOutput(0, 1.0);
    leftMaster.configSelectedFeedbackCoefficient(1.0 / 10.75);
    rightMaster.configMotionSCurveStrength(6);
    rightMaster.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_25Ms);
    rightMaster.configVelocityMeasurementWindow(7);

    zeroSensors();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  public void updateSensors() {
    mPeriodicIO.timestamp = Timer.getFPGATimestamp();

    mPeriodicIO.yaw_normalized = getHeading();
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

    m_odometry.update(Rotation2d.fromDegrees(mPeriodicIO.yaw_normalized), mPeriodicIO.left_pos_meters, mPeriodicIO.right_pos_meters);
  }

  public void setDriveStraight(double dist) {
    leftMaster.configMotionCruiseVelocity((int) (0.75 * DRIVE.kMaxNativeVel));
    leftMaster.configMotionAcceleration((int) (0.5 * DRIVE.kMaxNativeVel));
    rightMaster.configMotionCruiseVelocity((int) (0.75 * DRIVE.kMaxNativeVel));
    rightMaster.configMotionAcceleration((int) (0.50 * DRIVE.kMaxNativeVel));
    magicStraightTarget = dist;
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  public void updateDriveStraight() {
    leftMaster.set(ControlMode.MotionMagic, MkUtil.inchesToNative(magicStraightTarget));
    rightMaster.set(ControlMode.MotionMagic, MkUtil.inchesToNative(magicStraightTarget));
  }

  public boolean isDriveStraightDone() {
    double err = magicStraightTarget - mPeriodicIO.avg_dist_inches;
    if (Math.abs(err) < 0.5 && Math.abs(mPeriodicIO.avg_vel_inches_per_sec)  < 0.1) {
      leftMaster.set(ControlMode.PercentOutput, 0);
      rightMaster.set(ControlMode.PercentOutput, 0);
      return true;
    }
    return false;
  }

  public void setMagicTurnInPlace(double deg) {
    leftMaster.configMotionCruiseVelocity((int) (0.3 * DRIVE.kMaxNativeVel));
    leftMaster.configMotionAcceleration((int) (0.175 * DRIVE.kMaxNativeVel));
    rightMaster.configMotionCruiseVelocity((int) (0.3 * DRIVE.kMaxNativeVel));
    rightMaster.configMotionAcceleration((int) (0.175 * DRIVE.kMaxNativeVel));
    magicTarget = navX.getAngle() + deg;
  }

  public void magicTurnInPlaceUpdate() {
    double error_deg = mPeriodicIO.yaw_continouous - magicTarget;
    double error_rad = Math.toRadians(error_deg);//22.97
    double delta_v = 22.97 * error_rad / (2 * 0.95);
    leftMaster.set(ControlMode.MotionMagic, MkUtil.inchesToNative(-delta_v) + leftMaster.getSelectedSensorPosition());
    rightMaster.set(ControlMode.MotionMagic, MkUtil.inchesToNative(delta_v) + rightMaster.getSelectedSensorPosition());
    SmartDashboard.putNumber("Delta V Turn In Place", delta_v);
    SmartDashboard.putNumber("Left Set", leftMaster.getClosedLoopTarget());
  }

  public void magicTurnInPlaceUpdate(double ang){
    mPeriodicIO.yaw_continouous = navX.getAngle();
    magicTarget = ang + mPeriodicIO.yaw_continouous;
    magicTurnInPlaceUpdate();
  }

  public boolean isMagicOnTarget() {
    if (Math.abs(mPeriodicIO.yaw_continouous - magicTarget) < 3.0 && Math.abs(mPeriodicIO.avg_vel_inches_per_sec) < 0.1) {    
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

  public void setVoltage(double left, double right) {
    leftMaster.set(ControlMode.PercentOutput, (1.0 / 12.0) * left);
    rightMaster.set(ControlMode.PercentOutput, (1.0 / 12.0) * right);
  }

  public void setVelocityMetersPerSec(double leftVel, double rightVel) {
    leftMaster.set(ControlMode.Velocity, MkUtil.metersPerSecondToNativeUnitsPer100Ms(leftVel));
    rightMaster.set(ControlMode.Velocity, MkUtil.metersPerSecondToNativeUnitsPer100Ms(rightVel));
  }

  public double antiTip() {
    if (Math.abs(mPeriodicIO.roll) >= DRIVE.kAntiTipThreshold) {
      return Math.sin(Math.toRadians(navX.getRoll())) * -2;
    } else {
      return 0.0;
    }
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("NavX Roll", mPeriodicIO.roll);
    SmartDashboard.putNumber("NavX Yaw Normalized", mPeriodicIO.yaw_normalized);
    SmartDashboard.putNumber("NavX Yaw Continuous", mPeriodicIO.yaw_continouous);
    SmartDashboard.putNumber("Left Pos Meters", mPeriodicIO.left_pos_meters);
    SmartDashboard.putNumber("Right Pos Meters", mPeriodicIO.right_pos_meters);
    SmartDashboard.putNumber("Left Vel Meters/Sec", mPeriodicIO.left_vel_meters_per_sec);
    SmartDashboard.putNumber("Right Vel Meters/Sec", mPeriodicIO.right_vel_meters_per_sec);
    SmartDashboard.putNumber("Error Deg Turn In Place", mPeriodicIO.yaw_continouous - magicTarget);
    SmartDashboard.putBoolean("Magic Turn In Place Done", Math.abs(mPeriodicIO.yaw_continouous - magicTarget) < 3.0 && Math.abs(mPeriodicIO.avg_vel_inches_per_sec) < 0.1);
    SmartDashboard.putNumber("Pose X Inches", MkUtil.metersToInches(getPose().getTranslation().getX()));
    SmartDashboard.putNumber("Pose Y Inches", MkUtil.metersToInches(getPose().getTranslation().getY()));
    SmartDashboard.putNumber("Pose Theta Degrees", getPose().getRotation().getDegrees());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(mPeriodicIO.left_vel_meters_per_sec, mPeriodicIO.right_vel_meters_per_sec);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry() {
    m_odometry.resetPosition(new Pose2d(0,0,Rotation2d.fromDegrees(getHeading())), Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return -navX.getYaw();
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
    rollOffset = navX.getRoll();
    navX.zeroYaw();
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  public double getAvgVel(){
    return mPeriodicIO.avg_vel_inches_per_sec;
  }

  public double getYaw() {
    return navX.getAngle();
  }

  public static Drive getInstance() {
    return InstanceHolder.mInstance;
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
  }
}
