package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CenterAuto extends SequentialCommandGroup {
  public CenterAuto() {
    addCommands(
        deadline(new LimelightShoot().withTimeout(7)),
        new DriveStraight(18.0).withTimeout(3)
    );
  }
}
