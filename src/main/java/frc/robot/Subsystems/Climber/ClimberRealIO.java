package frc.robot.Subsystems.Climber;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberRealIO extends Climber {

    SparkMax climber_motor;
    
    protected ClimberRealIO() {
        climber_motor = new SparkMax(29, MotorType.kBrushless);
        
    }

    public void runClimber(double speed) {
        climber_motor.setVoltage(speed);
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Climber current", climber_motor.getOutputCurrent());
        SmartDashboard.putNumber("Climber volts real", climber_motor.getAppliedOutput());
        SmartDashboard.putNumber("Climber volts", climber_motor.getBusVoltage());

    }
    
}
