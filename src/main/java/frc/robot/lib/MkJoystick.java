package frc.robot.lib;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

public class MkJoystick extends Joystick {
  Timer holdTimer = new Timer();

  public MkJoystick(int port) {
    super(port);
    holdTimer.start();
  }

  public boolean getRawButtonPressed(int button, String name) {
    print(button, name);
    return getRawButtonPressed(button);
  }

  public boolean getRawButton(int button, String name) {
    if (holdTimer.hasElapsed(0.75)) {
      print(button, name);
      holdTimer.start();
    }
    return getRawButton(button);
  }

  public void print(int button, String name) {
    System.out.println("[Joystick] " + getPort() + "\t [Button]" + button + "\t Pressed ( " + name + " Button )");
  }
}
