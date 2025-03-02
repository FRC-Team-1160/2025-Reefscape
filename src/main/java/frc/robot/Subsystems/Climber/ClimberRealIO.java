package frc.robot.Subsystems.Climber;

import com.ctre.phoenix6.hardware.TalonFX;

public class ClimberRealIO extends Climber {

    TalonFX climber_motor;
    
    protected ClimberRealIO() {
        climber_motor = new TalonFX(12, "CANivore");
        
    }

    public void runClimber(double speed) {
        climber_motor.setVoltage(speed);
    }

    
}
