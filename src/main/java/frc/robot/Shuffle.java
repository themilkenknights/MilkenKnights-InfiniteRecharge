package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shuffle {

  private final AnalogInput pressure = new AnalogInput(0);
  private double lastTime = Timer.getFPGATimestamp();
  private int mUpdateDashboard;
  private int loopOverrunWarning;
  private int loopCounter;

  public static Shuffle getInstance() {
    return InstanceHolder.mInstance;
  }

  public void updateDeltaTime() {
    loopCounter++;
    double time = Timer.getFPGATimestamp();
    double dt = (time - lastTime) * 1e3;
    SmartDashboard.putNumber("Loop Dt", dt);
    lastTime = time;

    if (dt > 10) {
      loopOverrunWarning++;
    }

    if (loopCounter == 500) {
      if (loopOverrunWarning > 10) {
        System.out.println("Loop Time Over 22ms for more than 10 loops in the past 5 seconds.");
        loopCounter = 0;
      }
    }
  }

  public void update() {
    updateDeltaTime();
    mUpdateDashboard++;
    if (mUpdateDashboard == 4) {
      SmartDashboard.putString("Elevator Stopper", ElevatorStopper.getInstance().getStopperStateString());
      SmartDashboard.putNumber("Pressure", ((((pressure.getVoltage()) * 250 / 5.0 - 25.0) / 112) * 120));
      SmartDashboard.putNumber("Elevator Setpoint", Elevator.getInstance().getElevatorSetpoint());
      Drive.getInstance().updateDashboard();
      Limelight.getInstance().updateDashboard();
      Shooter.getInstance().updateDashboard();
      mUpdateDashboard = 0;
    }
  }

  private static class InstanceHolder {

    private static final Shuffle mInstance = new Shuffle();
  }
}
