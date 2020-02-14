package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

public class Limelight {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");

    private final double max_auto_output = 0.5;

    private final double kP_dist = 0.0; // First tune the turn PD loop
    private final double dist_tol = 0.5;
    private final double desired_vert_angle = 10;

    private final double kP_turn = 0.025;
    private final double kD_turn = kP_turn * 10;
    private final double angle_tol = 0.5;
    private final double max_anglular_vel = 15; // Deg/Sec
    private final double max_anglular_accel = 10; // Deg/Sec^2

    private final ProfiledPIDController m_controller = new ProfiledPIDController(kP_turn, 0.0, kD_turn,
            new TrapezoidProfile.Constraints(max_anglular_vel, max_anglular_accel));

    private Limelight() {

    }

    public void update() {
        // Get the horizantal and vertical angle we are offset by
        double horizantal_angle = tx.getDouble(0.0);
        double vertical_angle = ty.getDouble(0.0);

        // Goal angle - current angle
        double distance_error = desired_vert_angle - vertical_angle;
        double forward_output = 0;
        double turn_output = 0;

        // Have a deadband where we are close enough
        if (Math.abs(horizantal_angle) > angle_tol) {
            // Get PID controller output
            turn_output += m_controller.calculate(horizantal_angle);
        }

        // Have a deadband where we are close enough
        if (Math.abs(distance_error) > dist_tol) {
            // Just a proportional gain here
            forward_output += kP_dist * distance_error;
        }

        Drive.getInstance().setPercentOutput(deadband(forward_output + turn_output),
                deadband(forward_output - turn_output));
    }

    public double deadband(double a) {
        return Math.abs(a) < max_auto_output ? a : Math.copySign(max_auto_output, a);
    }

    public static Limelight getInstance() {
        return InstanceHolder.mInstance;
    }

    private static class InstanceHolder {
        private static final Limelight mInstance = new Limelight();
    }
}