package frc.robot.Subsystems.Climber;

import java.util.List;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class ClimberRealIO extends Climber {

    private TalonFX climber_motor;
    
    protected ClimberRealIO() {
        climber_motor = new TalonFX(13, "CANivore");        
    }

    public double runClimber(double speed) {
        climber_motor.setControl(new VoltageOut(speed));
        return speed;
    }

    public List<TalonFX> getTalons() {
        return List.of(climber_motor);
    }

    @Override
    public void periodic() {
        super.periodic();
    }
    
}
