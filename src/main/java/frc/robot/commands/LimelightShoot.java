package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Drive;
import frc.robot.Elevator;
import frc.robot.ElevatorStopper;
import frc.robot.Limelight;
import frc.robot.Shooter;
import frc.robot.ElevatorStopper.StopperState;
import frc.robot.lib.InterpolatingDouble;

public class LimelightShoot extends CommandBase {

    public LimelightShoot() {

    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Drive.getInstance().setOutput(Limelight.getInstance().update());
        double curDist = Limelight.getInstance().getDistance();
        double RPM = Constants.VISION.kRPMMap.getInterpolated(new InterpolatingDouble(curDist)).value;
        Shooter.getInstance().setShooterRPM(RPM);
        Shooter.getInstance().setHoodPos(curDist);
        if (Limelight.getInstance().inRange()) {
            Elevator.getInstance().setElevatorOutput(.420);
            ElevatorStopper.getInstance().setStopper(StopperState.GO);
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {

    }
}