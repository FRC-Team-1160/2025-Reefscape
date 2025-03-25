package frc.robot.Subsystems.Climber;

import java.util.List;

import com.ctre.phoenix6.hardware.TalonFX;

public class ClimberSimIO extends Climber {
    
    protected ClimberSimIO() {}

    public double runClimber(double speed) { return speed; }

    public List<TalonFX> getTalons() { return null; }

}
