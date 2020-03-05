package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RightAuto extends SequentialCommandGroup {
  public RightAuto() {
    addCommands(
        deadline(new DriveStraight(50.0).withTimeout(3), new SpinUp(3300)),
        deadline(new TurnInPlace(-30).withTimeout(3), new SpinUp(3300)),
        new LimelightShoot()
    );
  }
}
