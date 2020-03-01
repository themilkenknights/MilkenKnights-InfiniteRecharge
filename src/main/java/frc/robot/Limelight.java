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

  private final double kPDist = 0.005;
  private final double kDDist = .0005;

  private final double dist_tol = 0.5;

  private final double desiredDistance = 69;

  private final double kP_turn = 0.03;
  private final double kD_turn = .005;
  private final double angle_tol = 0.5;
  private final double maxVel = 19265.0; // Deg/Sec
  private final double maxAccel = 151917.25529762334; // Deg/Sec^2

  private final ProfiledPIDController m_controller = new ProfiledPIDController(kP_turn, 0.0, kD_turn,
      new TrapezoidProfile.Constraints(maxVel, maxAccel));

  private final ProfiledPIDController distance_controller = new ProfiledPIDController(kPDist, 0.0, kDDist,
      new TrapezoidProfile.Constraints(maxVel, maxAccel));

  private Limelight() {

  }

  public Drive.DriveSignal update() {
    // Get the horizantal and vertical angle we are offset by
    double horizantal_angle = tx.getDouble(0.0);
    double vertical_angle = ty.getDouble(0.0);
    double targetArea = ta.getDouble(0.0);

    SmartDashboard.putNumber("Horizantal Angle", tx.getDouble(0.0));
    SmartDashboard.putNumber("Vertical Angle", ty.getDouble(0.0));
    SmartDashboard.putNumber("Target Distance", getDistance());
    // Goal angle - current angle
    double distance_error = desiredDistance - getDistance();

    double forward_output = 0;
    double turn_output = 0;

    // Have a deadband where we are close enough
    if (Math.abs(horizantal_angle) > angle_tol) {
      // Get PID controller output
      m_controller.setGoal(0);
      turn_output = m_controller.calculate(-horizantal_angle);
    }

    // Have a deadband where we are close enough
    if (Math.abs(distance_error) > dist_tol) {
      // Just a proportional gain here
      forward_output = distance_controller.calculate(distance_error);
      forward_output= 0;
    }

    return new Drive.DriveSignal(deadband(forward_output + turn_output), deadband(forward_output - turn_output));
  }

  public double getDistance() {
    /*
     * double a = 98.6797 * Math.pow(ta.getDouble(0.0), -0.7282); double b = 9.4835
     * * Math.pow(0.9900, ta.getDouble(0.0));
     * 
     * return (a+b)/2;
     
    double a = Math.pow(.6988 * ta.getDouble(0.0), 2) + (-35.226 * ta.getDouble(0.0)) + 199.3863;
    */

    double a = Math.pow(Math.E, 5.5111 + -0.2701 * ta.getDouble(0.0));

    return a + 8;

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
