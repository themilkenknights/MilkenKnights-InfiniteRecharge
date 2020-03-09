package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shuffle {

  private final AnalogInput pressure = new AnalogInput(0);
  private double lastTime = Timer.getFPGATimestamp();
  private boolean mUpdateDashboard;
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

    if (dt > 22) {
      loopOverrunWarning++;
    }

    if (loopCounter == 500) {
      if (loopOverrunWarning > 10) {
        Shuffleboard.addEventMarker("Loop Time Over 22ms for more than 10 loops in the past 5 seconds.", EventImportance.kHigh);
        loopCounter = 0;
      }
    }
  }

  public void update() {
    updateDeltaTime();
    if (mUpdateDashboard) {
      SmartDashboard.putString("Elevator Stopper", ElevatorStopper.getInstance().getStopperStateString());
      SmartDashboard.putNumber("Pressure", ((((pressure.getVoltage()) * 250 / 5.0 - 25.0) / 112) * 120));
      SmartDashboard.putNumber("Elevator Setpoint", Elevator.getInstance().getElevatorSetpoint());
      Drive.getInstance().updateDashboard();
      Limelight.getInstance().updateDashboard();
      Shooter.getInstance().updateDashboard();
      mUpdateDashboard = false;
    } else {
      mUpdateDashboard = true;
    }
  }

  private static class InstanceHolder {

    private static final Shuffle mInstance = new Shuffle();
  }
}
