package frc.robot.Subsystems.Elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.PortConstants;
import frc.robot.Constants.ElevatorConstants.MotorConfigs;


public class ElevatorRealIO extends Elevator {

    public TalonFX left_motor, right_motor;
    
    SparkMax shooter_motor;

    public ElevatorRealIO() {
        left_motor = new TalonFX(PortConstants.LEFT_ELEVATOR_MOTOR, "CANivore");
        right_motor = new TalonFX(PortConstants.RIGHT_ELEVATOR_MOTOR, "CANivore");
        shooter_motor = new SparkMax(PortConstants.SHOOTER_MOTOR, MotorType.kBrushless);

        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.Slot0 = new Slot0Configs()
            .withKP(MotorConfigs.kP)
            .withKI(MotorConfigs.kI)
            .withKD(MotorConfigs.kD)
            .withKS(MotorConfigs.kS)
            .withKV(MotorConfigs.kV)
            .withKA(MotorConfigs.kA)
            .withKG(MotorConfigs.kG);

        configs.Feedback.SensorToMechanismRatio = 25;
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        right_motor.getConfigurator().apply(configs);
        left_motor.getConfigurator().apply(configs);

        setpoint = right_motor.getPosition().getValueAsDouble();
    }

    //DON'T ACTIVATE UNTIL FULLY TUNED
    public void setPIDControl() {
        // left_motor.setControl(new PositionVoltage(setpoint));
        // right_motor.setControl(new PositionVoltage(setpoint));    
    }

    public void setShooterSpeed(double speed) {
        shooter_motor.set(speed);
    }
}
