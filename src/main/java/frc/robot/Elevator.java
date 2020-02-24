package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Elevator {

  TalonSRX eTalon = new TalonSRX(Constants.CAN.ElevatorId);

  private Elevator() {
    eTalon.configFactoryDefault();
    eTalon.setInverted(Constants.CAN.elevatorInverted);
  }

  public void setElevatorOutput(double percentOut) {
    eTalon.set(ControlMode.PercentOutput, percentOut);
  }

  public static Elevator getInstance() {
    return InstanceHolder.mInstance;
  }

  private static class InstanceHolder {
    private static final Elevator mInstance = new Elevator();
  }
}