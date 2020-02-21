package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Elevator {

    TalonSRX eTalon = new TalonSRX(Constants.CAN.ElevatorId);
    TalonSRX hTalon = new TalonSRX(3);

    private Elevator(){
      eTalon.configFactoryDefault();
      hTalon.configFactoryDefault();
      eTalon.setInverted(Constants.CAN.elevatorInverted);
    }
    
  public void setElevatorOutput(double percentOut, double h) {
    eTalon.set(ControlMode.PercentOutput, percentOut);
    hTalon.set(ControlMode.PercentOutput, h);
  }

  public static Elevator getInstance() {
    return InstanceHolder.mInstance;
  }

  private static class InstanceHolder {
    private static final Elevator mInstance = new Elevator();
  }
}