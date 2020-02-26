package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shuffle
{

    public void Update()
    {

        SmartDashboard.putNumber("Shooter RPM", Shooter.getInstance().getShooterRPM());
    }
    
  public static Shuffle getInstance() {
    return InstanceHolder.mInstance;
  }

  private static class InstanceHolder {
    private static final Shuffle mInstance = new Shuffle();
  }
}