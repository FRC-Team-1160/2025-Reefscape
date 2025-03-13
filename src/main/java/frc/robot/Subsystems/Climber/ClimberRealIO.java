package frc.robot.Subsystems.Climber;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberRealIO extends Climber {

    TalonFX climber_motor;
    
    protected ClimberRealIO() {
        climber_motor = new TalonFX(13);
        
    }

    public void runClimber(double speed) {
        climber_motor.setControl(new VoltageOut(speed));
    }

    @Override
    public void periodic() {
        super.periodic();

    }
    
}
