package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Autonomous extends SequentialCommandGroup {
    /**
     * Create a new autonomous command.
     */
    public Autonomous() {
        addCommands(deadline(new DriveStraight(50), new IntakeBalls()), new TurnInPlace(180.0), new DriveStraight(50),
                new LimelightShoot());

    }
}