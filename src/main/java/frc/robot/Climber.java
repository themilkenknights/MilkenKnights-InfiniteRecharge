package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;

public class Climber {

    Solenoid Climber = new Solenoid(0);

    private Climber(){
        Climber.set(false);
    }

    public void toggleClimb() {
        if(Climber.get() == false)
            Climber.set(true);
        else
            Climber.set(false);
      }
    
    public static Climber getInstance() {
        return InstanceHolder.mInstance;
    }
    
    private static class InstanceHolder {
        private static final Climber mInstance = new Climber();
    }
}