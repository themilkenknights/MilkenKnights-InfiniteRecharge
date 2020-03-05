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

  public void updateT(){
    double time = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("Loop Dt", (time - lastTime) * 1e3);
    lastTime = time;
  }

  public void update() {
    SmartDashboard.putNumber("Shooter RPM", Shooter.getInstance().getShooterRPM());
    SmartDashboard.putNumber("Pressure", ((((pressure.getVoltage()) * 250 / 5.0 - 25.0) / 112) * 120));
    SmartDashboard.putNumber("Hood Pos", Shooter.getInstance().getHoodPos());
    SmartDashboard.putNumber("Hood Setpoint", Shooter.getInstance().getHoodPos());
    Drive.getInstance().updateDashboard();
    Limelight.getInstance().updateDashboard();
  }

  private static class InstanceHolder {

    private static final Shuffle mInstance = new Shuffle();
  }
}
