package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
        Intake.getInstance().setIntakeRoller(Constants.intakeSpeed);
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