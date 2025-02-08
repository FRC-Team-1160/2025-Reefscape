package frc.robot.Subsystems.Elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.PortConstants;
import frc.robot.Constants.ElevatorConstants.MotorConfigs;


public class ElevatorRealIO extends Elevator {

    public TalonFX left_ele_motor, right_ele_motor, wrist_motor;

    public SparkMax left_claw_motor, right_claw_motor, shooter_motor;

    public ElevatorRealIO() {
        left_ele_motor = new TalonFX(PortConstants.LEFT_ELEVATOR_MOTOR, "CANivore");
        right_ele_motor = new TalonFX(PortConstants.RIGHT_ELEVATOR_MOTOR, "CANivore");
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

        left_ele_motor.getConfigurator().apply(configs);
        right_ele_motor.getConfigurator().apply(configs);

        ele_setpoint = right_ele_motor.getPosition().getValueAsDouble();
    }

    public void setElevatorPID() {
        // left_motor.setControl(new PositionVoltage(setpoint));
        // right_motor.setControl(new PositionVoltage(setpoint));    
    }

    public void setLeftEleVoltage(double volts) {
        left_ele_motor.setControl(new VoltageOut(volts));
    }

    public void setRightEleVoltage(double volts) {
        right_ele_motor.setControl(new VoltageOut(volts));
    }

    //DON'T ACTIVATE UNTIL FULLY TUNED
    public void setLeftElePID(double setpoint) {
        // left_ele_motor.setControl(new PositionVoltage(setpoint));
    }

    public void setRightElePID(double setpoint) {
        // right_ele_motor.setControl(new PositionVoltage(setpoint));
    }

    public void setWristVoltage(double volts) {
        wrist_motor.setControl(new VoltageOut(volts));
    }

    public void setWristPID(double setpoint) {
        wrist_motor.setControl(new PositionVoltage(setpoint));
    }

    public void setShooterSpeed(double speed) {
        shooter_motor.set(speed);
    }
}
