package frc.robot.lib;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants;

public class PressureSensor {
  private final AnalogInput mAnalogInput;

  public PressureSensor(int analogInputNumber) {
    mAnalogInput = new AnalogInput(analogInputNumber);
  }

  public double getAirPressurePsi() {
    return Constants.kPressureMap.getInterpolated(
            new InterpolatingDouble(mAnalogInput.getVoltage()))
        .value;
  }

  public double getVoltage() {
    return mAnalogInput.getVoltage();
  }
}
