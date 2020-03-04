package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Autonomous extends SequentialCommandGroup {
    /**
     * Create a new autonomous command.
     */
    public Autonomous() {
        addCommands(
        deadline(new DriveStraight(70.0), new IntakeBalls()),
         deadline(new TurnInPlace(-90.0), new IntakeBalls()),
         deadline(new DriveStraight(10), new IntakeBalls()),
          deadline(new TurnInPlace(-90.0), new IntakeBalls()),
            new DriveStraight(65.5),  new TurnInPlace(-40.0)
                /*new LimelightShoot(),*/
    );

    }
}