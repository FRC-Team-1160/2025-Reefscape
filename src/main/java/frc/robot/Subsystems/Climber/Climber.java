package frc.robot.Subsystems.Climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

abstract public class Climber extends SubsystemBase {
    
    public static final Climber instance = Robot.isReal() ? new ClimberRealIO() : new ClimberSimIO();

    protected Climber() {

    }

    public abstract void runClimber(double speed);
}
