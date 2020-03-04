package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shuffle {
  AnalogInput pressure = new AnalogInput(0);

  public static Shuffle getInstance() {
    return InstanceHolder.mInstance;
  }

  public void Update() {
    SmartDashboard.putNumber("Shooter RPM", Shooter.getInstance().getShooterRPM());
    SmartDashboard.putNumber("Distance", Limelight.getInstance().getDistance());
    SmartDashboard.putNumber("Pressure", ( (((pressure.getVoltage()) * 250 / 5.0 - 25.0)/112) * 120));
    SmartDashboard.putNumber("Rot Vel", Drive.getInstance().getYawVel());
    SmartDashboard.putNumber("Yaw", Drive.getInstance().getYaw());
    SmartDashboard.putNumber("RPos", Drive.getInstance().getPos());
    SmartDashboard.putNumber("RVel", Drive.getInstance().getVelocity());
    SmartDashboard.putNumber("RPosIn", Drive.getInstance().nativeUnitsToInches(Drive.getInstance().getPos()));
    SmartDashboard.putNumber("Hood Pos", Shooter.getInstance().getHoodPos());

  }

  private static class InstanceHolder {

    private static final Shuffle mInstance = new Shuffle();
  }
}
