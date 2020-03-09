package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Elevator {

  private final TalonSRX mTalon = new TalonSRX(Constants.CAN.kElevatorId);
  private double elevatorSetpoint;

  private Elevator() {
    mTalon.configFactoryDefault();
    mTalon.configVoltageCompSaturation(12.0);
    mTalon.enableVoltageCompensation(true);
    mTalon.setNeutralMode(NeutralMode.Brake);
    mTalon.setInverted(Constants.CAN.kElevatorInverted);
  }

  public static Elevator getInstance() {
    return InstanceHolder.mInstance;
  }

  public void setElevatorOutput(double ePercentOut) {
    mTalon.set(ControlMode.PercentOutput, ePercentOut);
    elevatorSetpoint = ePercentOut;
  }

  public double getElevatorSetpoint() {
    return elevatorSetpoint;
  }

  private static class InstanceHolder {

    private static final Elevator mInstance = new Elevator();
  }
}
