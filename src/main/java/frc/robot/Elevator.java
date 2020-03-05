package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Elevator {

  private final TalonSRX eTalon = new TalonSRX(Constants.CAN.kElevatorId);
  private double elevatorSetpoint;

  private Elevator() {
    eTalon.configFactoryDefault();
    eTalon.configVoltageCompSaturation(12.0);
    eTalon.enableVoltageCompensation(true);
    eTalon.setNeutralMode(NeutralMode.Brake);
    eTalon.setInverted(Constants.CAN.kElevatorInverted);
  }

  public static Elevator getInstance() {
    return InstanceHolder.mInstance;
  }

  public void setElevatorOutput(double ePercentOut) {
    eTalon.set(ControlMode.PercentOutput, ePercentOut);
    elevatorSetpoint = ePercentOut;
  }

  public double getElevatorSetpoint() {
    return elevatorSetpoint;
  }

  private static class InstanceHolder {

    private static final Elevator mInstance = new Elevator();
  }
}
