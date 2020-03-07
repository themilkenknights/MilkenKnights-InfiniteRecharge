package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class LeftTrenchAuto extends SequentialCommandGroup {
  public LeftTrenchAuto() {
    addCommands(
        deadline(new DriveStraight(98.0).withTimeout(3), new IntakeBalls()),
        deadline(new DriveStraight(-10.0).withTimeout(3), new IntakeBalls()),
        deadline(new TurnInPlace(-120.0).withTimeout(2.5), new IntakeBalls()),
        deadline(new DriveStraight(115).withTimeout(3), new IntakeBalls(), new SpinUp(3600)),
        deadline(new TurnInPlace(-30.0).withTimeout(1.5), new IntakeBalls(), new SpinUp(3600)),
        new LimelightShoot()
    );
  }
}
