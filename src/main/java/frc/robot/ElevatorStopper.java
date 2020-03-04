/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;

public class ElevatorStopper {

  private final Solenoid stopperSolenoid = new Solenoid(2);

  public ElevatorStopper() {
    stopperSolenoid.set(false);
  }

  public static ElevatorStopper getInstance() {
    return InstanceHolder.mInstance;
  }

  public void toggleStopper() {
    if (stopperSolenoid.get()) {
      stopperSolenoid.set(false);
    } else {
      stopperSolenoid.set(true);
    }
  }

  public void setStopper(StopperState state) {
    stopperSolenoid.set(state.state);
  }

  public enum StopperState {
    GO(true), STOP(false);

    public final boolean state;

    StopperState(final boolean state) {
      this.state = state;
    }
  }

  private static class InstanceHolder {

    private static final ElevatorStopper mInstance = new ElevatorStopper();
  }
}
