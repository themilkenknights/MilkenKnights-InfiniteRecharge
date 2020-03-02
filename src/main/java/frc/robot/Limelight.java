package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

public class Limelight {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  private final double max_auto_output = 0.5;

  private final double kP_dist = 0.005;
  private final double kD_dist = .0005;
  private final double dist_tol = 0.5;
  private final double max_dist = 75;
  private final double min_dist = 60;
  private final double max_linear_vel = 100;
  private final double max_linear_accel = 140;

  private final double kP_turn = 0.03;
  private final double kD_turn = .005;
  private final double angle_tol = 0.3;
  private final double max_angular_vel = 19265.0; // Deg/Sec @TODO These are definitely way too high. Try something like 1200 Deg/Sec
  private final double max_angular_accel = 151917.25529762334; // Deg/Sec^2 Try 2000 Deg/Sec^2

  private final ProfiledPIDController m_turn_controller = new ProfiledPIDController(kP_turn, 0.0, kD_turn,
      new TrapezoidProfile.Constraints(max_angular_vel, max_angular_accel));

  private final ProfiledPIDController m_dist_controller = new ProfiledPIDController(kP_dist, 0.0, kD_dist,
      new TrapezoidProfile.Constraints(max_angular_vel, max_angular_accel));

  private Limelight() {
    m_turn_controller.setTolerance(angle_tol);
    m_dist_controller.setTolerance(dist_tol);
  }

  public boolean inRange() {
    return m_dist_controller.atGoal() && m_turn_controller.atGoal();
  }

  public Drive.DriveSignal update() {
    // Get the horizontal and vertical angle we are offset by
    double horizontal_angle = tx.getDouble(0.0);
    double vertical_angle = ty.getDouble(0.0);

    SmartDashboard.putNumber("Horizontal Angle", horizontal_angle);
    SmartDashboard.putNumber("Vertical Angle", vertical_angle);
    SmartDashboard.putNumber("Target Distance", getDistance());
    // Goal angle - current angle

    double forward_output = 0;
    double turn_output = 0;

    // Have a deadband where we are close enough
    if (Math.abs(horizontal_angle) > angle_tol) {
      // Get PID controller output
      turn_output = m_turn_controller.calculate(-horizontal_angle, 0);
    }

    // Have a deadband where we are close enough
    double current_dist = getDistance();
    if (current_dist > max_dist) {
      forward_output = m_dist_controller.calculate(current_dist, max_dist);
    } else if (getDistance() < min_dist) {
      forward_output = m_dist_controller.calculate(current_dist, min_dist);
    }

    return new Drive.DriveSignal(clamp(forward_output + turn_output), clamp(forward_output - turn_output));
  }

  public double getDistance() {
    /* double a = 98.6797 * Math.pow(ta.getDouble(0.0), -0.7282); double b = 9.4835
    Math.pow(0.9900, ta.getDouble(0.0));
    return (a+b)/2;
    double a = Math.pow(.6988 * ta.getDouble(0.0), 2) + (-35.226 * ta.getDouble(0.0)) + 199.3863;
    double a = Math.pow(Math.E, 5.5111 + -0.2701 * ta.getDouble(0.0));
    return a + 8; */

    double height_camera_to_target = (89.75 - 33); //inches
    double lightlight_pitch = 20.0; //Degrees from horizontal

    return (height_camera_to_target / Math.tan(Math.toRadians(lightlight_pitch + ty.getDouble(0.0)))) - 17.5;
  }

  public double clamp(double a) {
    return Math.abs(a) < max_auto_output ? a : Math.copySign(max_auto_output, a);
  }

  public static Limelight getInstance() {
    return InstanceHolder.mInstance;
  }

  private static class InstanceHolder {
    private static final Limelight mInstance = new Limelight();
  }
}
