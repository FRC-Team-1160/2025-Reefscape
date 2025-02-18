package frc.robot.Subsystems.Elevator;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.PortConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorConfigs;
import frc.robot.Constants.ElevatorConstants.WristConfigs;


public class ElevatorRealIO extends Elevator {

    private TalonFX ele_motor, wrist_motor;

    private SparkMax claw_motor_left, claw_motor_right, shooter_motor;

    public ElevatorRealIO() {
        ele_motor = new TalonFX(PortConstants.RIGHT_ELEVATOR_MOTOR, "CANivore");
        shooter_motor = new SparkMax(PortConstants.SHOOTER_MOTOR, MotorType.kBrushless);

        TalonFXConfiguration ele_configs = new TalonFXConfiguration();
        ele_configs.Slot0 = new Slot0Configs()
            .withKP(ElevatorConfigs.kP)
            .withKI(ElevatorConfigs.kI)
            .withKD(ElevatorConfigs.kD)
            .withKS(ElevatorConfigs.kS)
            .withKV(ElevatorConfigs.kV)
            .withKA(ElevatorConfigs.kA)
            .withKG(ElevatorConfigs.kG);

        ele_configs.Feedback.SensorToMechanismRatio = 25;
        ele_configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        TalonFXConfiguration wrist_configs = new TalonFXConfiguration();
        wrist_configs.Slot0 = new Slot0Configs()
            .withKP(WristConfigs.kP)
            .withKI(WristConfigs.kI)
            .withKD(WristConfigs.kD)
            .withKS(WristConfigs.kS)
            .withKV(WristConfigs.kV)
            .withKA(WristConfigs.kA)
            .withKG(WristConfigs.kG);

        wrist_motor.getConfigurator().apply(wrist_configs);

        ele_setpoint = ele_motor.getPosition().getValueAsDouble();
        wrist_setpoint = wrist_motor.getPosition().getValueAsDouble();

    }

    public void runElevator(double speed) {
        ele_motor.setControl(new VoltageOut(-(speed + Math.signum(speed) * ElevatorConfigs.kS) + ElevatorConfigs.kG));
    }

    public void runWrist(double speed) {
        wrist_motor.setControl(new VoltageOut(speed + Math.signum(speed) * WristConfigs.kS));
    }

    //DON'T ACTIVATE UNTIL FULLY TUNED
    protected void setElePID(double setpoint){
        // ele_motor.setControl(new PositionVoltage(setpoint));
    }

    //DON'T ACTIVATE UNTIL FULLY TUNED
    protected void setWristPID(double setpoint) {
        // wrist_motor.setControl(new PositionVoltage(setpoint));
    }

    protected void setLeftClawSpeed(double speed) {
        claw_motor_left.set(speed);
    }

    protected void setRightClawSpeed(double speed) {
        claw_motor_right.set(speed);
    }

    protected void setShooterSpeed(double speed) {
        shooter_motor.set(speed);
    }

    public List<TalonFX> getTalons() {
        return Arrays.asList(ele_motor, wrist_motor);
    }
}
