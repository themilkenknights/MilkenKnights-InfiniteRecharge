package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CenterAuto extends SequentialCommandGroup {
  public CenterAuto() {
    addCommands(
        //deadline(new DriveStraight(98.0).withTimeout(3), new SpinUp(3200)),
        new LimelightShoot()
    );
  }
}
