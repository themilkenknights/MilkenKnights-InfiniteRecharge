package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Drive {

    private TalonFX rightMaster = new TalonFX(Constants.Drive.rightMasterId);
    private TalonFX leftMaster = new TalonFX(Constants.Drive.leftMasterId);
    private TalonFX rightSlave = new TalonFX(Constants.Drive.rightSlaveId);
    private TalonFX leftSlave = new TalonFX(Constants.Drive.leftSlaveId);

    private Drive() {
        rightMaster.configFactoryDefault();
        leftMaster.configFactoryDefault();
        rightSlave.configFactoryDefault();
        leftSlave.configFactoryDefault();

        rightMaster.setInverted(true);
        leftMaster.setInverted(false);
        rightSlave.setInverted(true);
        leftSlave.setInverted(false);

        rightMaster.setSensorPhase(false);
        leftMaster.setSensorPhase(false);

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    }

    public void setPercentOutput(double left, double right) {
        leftMaster.set(ControlMode.PercentOutput, left);
        rightMaster.set(ControlMode.PercentOutput, right);
    }

    public double getDistance() {
        return nativeUnitsToInches(
                (rightMaster.getSelectedSensorPosition() + leftMaster.getSelectedSensorPosition()) / 2.0);
    }

    private double nativeUnitsToInches(double nativeUnits) {
        return (nativeUnits / 4096.0) * Constants.Drive.pi * Constants.Drive.wheelDiameterInches;
    }

    public static Drive getInstance() {
        return InstanceHolder.mInstance;
    }

    private static class InstanceHolder {
        private static final Drive mInstance = new Drive();
    }

}