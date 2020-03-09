package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class BackAuto extends SequentialCommandGroup {
  public BackAuto() {
    addCommands(
        deadline(new LimelightShoot().withTimeout(7)),
        new DriveStraight(-18.0).withTimeout(3)
    );
  }
}
