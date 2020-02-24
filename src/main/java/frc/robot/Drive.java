package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.Constants.DRIVE;
import frc.robot.Constants.CAN;

public class Drive {
  private TalonFX leftMaster = new TalonFX(CAN.driveLeftMasterId);
  private TalonFX leftSlave = new TalonFX(CAN.driveLeftSlaveId);
  private TalonFX rightMaster = new TalonFX(CAN.driveRightMasterId);
  private TalonFX rightSlave = new TalonFX(CAN.driveRightSlaveId);
  private static AHRS navX = new AHRS();

  private Drive() {
    leftMaster.configFactoryDefault();
    leftSlave.configFactoryDefault();
    rightMaster.configFactoryDefault();
    rightSlave.configFactoryDefault();

    leftMaster.setInverted(DRIVE.leftMasterInverted);
    leftSlave.setInverted(DRIVE.leftSlaveInverted);
    rightMaster.setInverted(DRIVE.rightMasterInverted);
    rightSlave.setInverted(DRIVE.rightSlaveInverted);

    leftMaster.setSensorPhase(DRIVE.leftSensorInverted);
    rightMaster.setSensorPhase(DRIVE.rightSensorInverted);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    leftMaster.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);
  }

  public void setOutput(DriveSignal signal) {
    leftMaster.set(ControlMode.PercentOutput, signal.getLeft());
    rightMaster.set(ControlMode.PercentOutput, signal.getRight());
  }

  public double getAverageDistance() {
    return nativeUnitsToInches(
        (rightMaster.getSelectedSensorPosition() + leftMaster.getSelectedSensorPosition()) / 2.0);
  }
  

  public double AntiTip() {
    double roll = navX.getPitch();

    if(roll >= Math.abs(Constants.DRIVE.AngleThresholdDegrees))
    {
      double rollAngleRadians = Constants.DRIVE.AngleThresholdDegrees * (Math.PI / 180.0);
      return Math.sin(rollAngleRadians) * -1;
    }
    else
      return 0.0;
  }

  private double nativeUnitsToInches(double nativeUnits) {
    return (nativeUnits / 4096.0) * DRIVE.wheelCircumference;
  }

  public static Drive getInstance() {
    return InstanceHolder.mInstance;
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
