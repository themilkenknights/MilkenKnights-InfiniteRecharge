package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DRIVE;

public class Drive {

  private AHRS navX = new AHRS();
  private TalonFX leftMaster = new TalonFX(CAN.driveLeftMasterId);
  private TalonFX leftSlave = new TalonFX(CAN.driveLeftSlaveId);
  private TalonFX rightMaster = new TalonFX(CAN.driveRightMasterId);
  private TalonFX rightSlave = new TalonFX(CAN.driveRightSlaveId);
  private double rollOffset, lastVel, lastTime;

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

    rollOffset = navX.getRoll();
    //TODO: Verify you want this behavior. If you are carrying the robot while it boots up this value will be erroneous
  }

  public static Drive getInstance() {
    return InstanceHolder.mInstance;
  }

  public double getVelocity() {
    return rightMaster.getSelectedSensorVelocity();
  }

  public double getAcceleration() {
    double curVel = rightMaster.getSelectedSensorVelocity();
    double curTime = Timer.getFPGATimestamp();
    double acc = (curVel - lastVel) / (Timer.getFPGATimestamp() - lastTime);
    lastVel = curVel;
    lastTime = curTime;
    return acc;
  }

  public void setOutput(DriveSignal signal) {
    leftMaster.set(ControlMode.PercentOutput, signal.getLeft());
    rightMaster.set(ControlMode.PercentOutput, signal.getRight());
  }

  public double getAverageDistance() {
    return nativeUnitsToInches((rightMaster.getSelectedSensorPosition() + leftMaster.getSelectedSensorPosition()) / 2.0);
  }

  public double getRoll() {
    return navX.getRoll() - rollOffset;
  }

  public double antiTip() {
    double roll = getRoll();
    // Simplified Logic Below
    if (Math.abs(roll) >= Constants.DRIVE.AngleThresholdDegrees) {
      return Math.sin(Math.toRadians(roll)) * -2;
    } else {
      return 0.0;
    }
  }

  private double nativeUnitsToInches(double nativeUnits) {
    return (nativeUnits / 4096.0) * DRIVE.wheelCircumference;
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
