package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shuffle {

  private final AnalogInput pressure = new AnalogInput(0);
  private double lastTime = Timer.getFPGATimestamp();

  public static Shuffle getInstance() {
    return InstanceHolder.mInstance;
  }

  public void updateDeltaTime() {
    double time = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("Loop Dt", (time - lastTime) * 1e3);
    lastTime = time;
  }

  public void update() {
    SmartDashboard.putString("Elevator Stopper", ElevatorStopper.getInstance().getStopperState());
    SmartDashboard.putNumber("Pressure", ((((pressure.getVoltage()) * 250 / 5.0 - 25.0) / 112) * 120));
    SmartDashboard.putNumber("Elevator Setpoint", Elevator.getInstance().getElevatorSetpoint());
    Drive.getInstance().updateDashboard();
    Limelight.getInstance().updateDashboard();
    Shooter.getInstance().updateDashboard();
  }

  private static class InstanceHolder {

    private static final Shuffle mInstance = new Shuffle();
  }
}
