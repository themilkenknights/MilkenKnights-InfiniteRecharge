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
    double dt = (time - lastTime) * 1e3;
    SmartDashboard.putNumber("Loop Dt", dt);
    lastTime = time;
    if (dt > 40) {
      System.out.println("If you see this message appear a bunch in a row contact swerdlow");
    }
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
