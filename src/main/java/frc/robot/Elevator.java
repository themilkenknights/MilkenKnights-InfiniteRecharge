package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Elevator {

  TalonSRX eTalon = new TalonSRX(Constants.CAN.ElevatorId);
  CANSparkMax hopperSparkMax = new CANSparkMax(Constants.CAN.HopperId, MotorType.kBrushless);

  private Elevator() {
    eTalon.configFactoryDefault();
    eTalon.setInverted(Constants.CAN.elevatorInverted);
  }

  public void setElevatorOutput(double ePercentOut, double hPercentOut) {
    eTalon.set(ControlMode.PercentOutput, ePercentOut);
    hopperSparkMax.set(hPercentOut);
  }

  public static Elevator getInstance() {
    return InstanceHolder.mInstance;
  }

  private static class InstanceHolder {
    private static final Elevator mInstance = new Elevator();
  }
}