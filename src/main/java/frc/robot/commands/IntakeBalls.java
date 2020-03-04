package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Elevator;
import frc.robot.ElevatorStopper;
import frc.robot.ElevatorStopper.StopperState;
import frc.robot.Intake;

public class IntakeBalls extends CommandBase {

    public IntakeBalls() {
       
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Intake.getInstance().setIntakeState(true);
    Intake.getInstance().setIntakeRoller(.75);
    Elevator.getInstance().setElevatorOutput(0.20);
    Intake.getInstance().setHopperRoller(.42);
    ElevatorStopper.getInstance().setStopper(StopperState.STOP);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {

    }
}