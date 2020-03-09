package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class LeftTrenchAuto extends SequentialCommandGroup {
  public LeftTrenchAuto() {
    addCommands(
        deadline(new DriveStraight(88.0).withTimeout(3), new IntakeBalls()),
        deadline(new DriveStraight(-15.0).withTimeout(3), new IntakeBalls()),
        deadline(new TurnInPlace(-135.0).withTimeout(2.5), new IntakeBalls().withTimeout(2.5)),
        deadline(new DriveStraight(95.0).withTimeout(3), new SpinUp(3600)),
        deadline(new TurnInPlace(20.0).withTimeout(1.5), new SpinUp(3600)),
        new LimelightShoot()
    );
  }
}
