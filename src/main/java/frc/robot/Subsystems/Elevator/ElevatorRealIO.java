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

    public TalonFX ele_motor, wrist_motor;

    public SparkMax left_claw_motor, right_claw_motor, shooter_motor;

    public ElevatorRealIO() {
        ele_motor = new TalonFX(PortConstants.RIGHT_ELEVATOR_MOTOR, "CANivore");
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

        ele_motor.getConfigurator().apply(configs);

        ele_setpoint = ele_motor.getPosition().getValueAsDouble();
    }

    protected void setEleVoltage(double volts) {
        ele_motor.setControl(new VoltageOut(volts));
    }

    protected void setWristVoltage(double volts) {
        wrist_motor.setControl(new VoltageOut(volts));
    }

    //DON'T ACTIVATE UNTIL FULLY TUNED
    protected void setElePID(double setpoint){
        // ele_motor.setControl(new PositionVoltage(setpoint));
    }

    protected void setWristPID(double setpoint) {
        wrist_motor.setControl(new PositionVoltage(setpoint));
    }

    protected void setLeftClawSpeed(double speed) {
        left_claw_motor.set(speed);
    }

    protected void setRightClawSpeed(double speed) {
        right_claw_motor.set(speed);
    }

    public void setShooterSpeed(double speed) {
        shooter_motor.set(speed);
    }
}
