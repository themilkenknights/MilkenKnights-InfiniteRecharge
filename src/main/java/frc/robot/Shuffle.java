package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shuffle {

  public void Update() {
    AnalogInput pressure = new AnalogInput(0);
    SmartDashboard.putNumber("Shooter RPM", Shooter.getInstance().getShooterRPM());
    SmartDashboard.putNumber("Distance", Limelight.getInstance().getDistance());
    SmartDashboard.putNumber("Pressure", ( (pressure.getVoltage()) * 250 / 5.0 - 25.0));
  }

  public static Shuffle getInstance() {
    return InstanceHolder.mInstance;
  }

  private static class InstanceHolder {
    private static final Shuffle mInstance = new Shuffle();
  }
}