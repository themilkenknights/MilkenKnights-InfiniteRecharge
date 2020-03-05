package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants.VISION;
import frc.robot.lib.InterpolatingDouble;
import frc.robot.lib.MkUtil.DriveSignal;

public class Limelight {

  private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(VISION.max_angular_vel, VISION.max_angular_accel);
  private final ProfiledPIDController m_turn_controller = new ProfiledPIDController(VISION.kP_turn, VISION.kI_turn, VISION.kD_turn, constraints);
  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private final NetworkTableEntry tx = table.getEntry("tx");
  private final NetworkTableEntry ty = table.getEntry("ty");
  private final NetworkTableEntry led = table.getEntry("ledMode");
  private final Timer shootTimer = new Timer();
  private boolean shootOn = false;
  private double distance, visionYaw, visionPitch;

  private Limelight() {
    m_turn_controller.setTolerance(VISION.angle_tol);
    m_turn_controller.setIntegratorRange(-100.0, 100.0);
  }

  public void updateSensors() {
    visionYaw = tx.getDouble(0.0);
    visionPitch = ty.getDouble(0.0);
    distance = getDistance(visionPitch);
  }


  public boolean inRange() {
    return Math.abs(visionYaw) < Constants.VISION.angle_tol && Math.abs(Drive.getInstance().getAvgVel()) < 0.1;
  }

  public void autoAimShoot(double limelightOffset) {
    Intake.getInstance().setIntakeRoller(0.0);
    Intake.getInstance().setIntakeState(Intake.IntakeState.STOW);
    update();
    double curDist = getDistance();
    double RPM = Constants.VISION.kRPMMap.getInterpolated(new InterpolatingDouble(curDist)).value;
    double hoodDist = Constants.VISION.kHoodMap.getInterpolated(new InterpolatingDouble(curDist)).value;
    Shooter.getInstance().setShooterRPM(RPM);
    Shooter.getInstance().setHoodPos(hoodDist); //Add limelight offset
    Elevator.getInstance().setElevatorOutput(0.79 - Constants.VISION.elevatorSlope * Limelight.getInstance().getDistance());
    SmartDashboard.putNumber("Map RPM", RPM);
    SmartDashboard.putNumber("Map Hood Pos", hoodDist);
    SmartDashboard.putNumber("Elevator",0.79 - Constants.VISION.elevatorSlope * Limelight.getInstance().getDistance());
    boolean isInRange = inRange();
    SmartDashboard.putBoolean("In Range", isInRange);
    if (isInRange && Math.abs(Shooter.getInstance().getShooterRPM() - RPM) < 30) {
      ElevatorStopper.getInstance().setStopper(ElevatorStopper.StopperState.GO);
      shootTimer.start();
      shootOn = true;
    } else if (shootTimer.hasElapsed(0.25) || !shootOn) {
      shootTimer.reset();
      shootOn = false;
      ElevatorStopper.getInstance().setStopper(ElevatorStopper.StopperState.STOP);
    }
  }

  public void update() {


double horizontal_angle = tx.getDouble(0.0);
   // Drive.getInstance().magicTurnInPlaceUpdate(visionYaw);
double turn_output = 0;
    // Have a deadband where we are close enough
   if (Math.abs(horizontal_angle) > VISION.angle_tol) {
      // Get PID controller output
      TrapezoidProfile trap = new TrapezoidProfile(constraints, new TrapezoidProfile.State(0, 0));
      turn_output = clamp(m_turn_controller.calculate(-horizontal_angle, 0) + ((1.0) / (VISION.max_angular_vel)) * trap.calculate(Constants.kDt).velocity);
    }

  Drive.getInstance().setOutput(new DriveSignal(turn_output, -turn_output));
  }

  public double getDistance() {
    return distance;
  }

  public void toggleLED(){
      if(led.getDouble(0.0) == 1){
        table.getEntry("ledMode").setValue(3);
      } else{
        table.getEntry("ledMode").setValue(1);
      }
  }

  public double getDistance(double ty) {
    double height_camera_to_target = (89.75 - 33); //inches
    double lightlight_pitch = 17.7; //Degrees from horizontal
    return (height_camera_to_target / Math.tan(Math.toRadians(lightlight_pitch + ty)));
  }

  public double clamp(double a) {
    return Math.abs(a) < VISION.max_auto_output ? a : Math.copySign(VISION.max_auto_output, a);
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("Horizontal Angle", visionYaw);
    SmartDashboard.putNumber("Vertical Angle", visionPitch);
    SmartDashboard.putNumber("Target Distance", distance);
  }

  public static Limelight getInstance() {
    return InstanceHolder.mInstance;
  }

  private static class InstanceHolder {

    private static final Limelight mInstance = new Limelight();
  }
}
