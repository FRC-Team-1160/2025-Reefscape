package frc.robot.Subsystems.Climber;

import java.util.List;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Robot;

/** The climber subsystem. */
abstract public class Climber extends SubsystemBase {
    
    public static final Climber instance = Robot.isReal() ? new ClimberRealIO() : new ClimberSimIO();

    /** Class constructor. */
    protected Climber() {

    }

    /**
     * Runs the climber at a set speed through voltage.
     * @param speed The voltage at which to run the motor.
     */
    @AutoLogOutput
    public abstract double runClimber(double speed);

    public abstract List<TalonFX> getTalons();
}
