package frc.robot.Subsystems.Elevator;

import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotionMagicIsRunningValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.RobotUtils;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorConfigs;
import frc.robot.Constants.ElevatorConstants.ElevatorMotionMagic;


public class ElevatorRealIO extends Elevator {

    protected TalonFX ele_motor;

    private SparkMax algae_motor, wrist_motor, shooter_motor;

    private DigitalInput elevator_switch, coral_switch;

    private boolean update_zero;

    private double count;

    protected ElevatorRealIO() {
        ele_motor = new TalonFX(PortConstants.ELEVATOR_MOTOR, "CANivore");
        
        wrist_motor = new SparkMax(PortConstants.WRIST_MOTOR, MotorType.kBrushless);
        algae_motor = new SparkMax(PortConstants.ALGAE_MOTOR, MotorType.kBrushless);
        shooter_motor = new SparkMax(PortConstants.SHOOTER_MOTOR, MotorType.kBrushless);

        elevator_switch = new DigitalInput(1);
        coral_switch = new DigitalInput(0);

        TalonFXConfiguration ele_configs = new TalonFXConfiguration();
        ele_configs.Slot0 = new Slot0Configs()
            .withKP(ElevatorConfigs.kP)
            .withKI(ElevatorConfigs.kI)
            .withKD(ElevatorConfigs.kD)
            .withKS(ElevatorConfigs.kS)
            .withKV(ElevatorConfigs.kV)
            .withKA(ElevatorConfigs.kA)
            .withKG(ElevatorConfigs.kG);

        ele_configs.MotionMagic = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(ElevatorMotionMagic.VELOCITY)
            .withMotionMagicAcceleration(ElevatorMotionMagic.ACCELERATION)
            .withMotionMagicJerk(ElevatorMotionMagic.JERK)
            .withMotionMagicExpo_kV(ElevatorMotionMagic.EXPO_kV)
            .withMotionMagicExpo_kA(ElevatorMotionMagic.EXPO_kA);   

        ele_configs.Feedback.SensorToMechanismRatio = 15;
        ele_configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        ele_configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        ele_motor.getConfigurator().apply(ele_configs);

        update_zero = true;
        count = 0;
    }

    public double runElevator(double speed) {
        ele_motor.setControl(new VoltageOut(speed));
        return speed;
    }

    public double runWrist(double speed) {
        Logger.recordOutput("Elevator/runningWrist", speed);
        wrist_motor.setVoltage(speed);
        return speed;
    }

    public Command setWristCmd(boolean up) {
        return RobotUtils.onOffCommand(this::runWrist, up ? 2 : -1.5, up ? 0 : 0).withTimeout(0.5);
    }

    protected void runEleMotionMagic(double setpoint){
        ele_motor.setControl(new MotionMagicVoltage(setpoint).withFeedForward(setpoint >= 5 ? 0.2 : 0));
    }

    public void stopElevator(){
        runElevator(0.35);
    }

    public void runAlgae(double speed) {
        algae_motor.set(speed);
    }

    public void runShooter(double speed) {
        shooter_motor.set(-speed);
    }

    public double getElevatorHeight() {
        return ele_motor.getPosition().getValueAsDouble();
    }

    public boolean atSetpoint() {
        return Math.abs(ele_motor.getClosedLoopError().getValueAsDouble()) < 0.02;
    }

    public List<TalonFX> getTalons() {
        return Arrays.asList(ele_motor);
    }

    public boolean getCoralStored() {
        return !coral_switch.get();
    }

    @AutoLogOutput
    public boolean getElevatorZeroed() {
        return !elevator_switch.get();
    }

    @Override
    public void periodic() {
        super.periodic();

        Logger.recordOutput("Elevator/Wrist Current", wrist_motor.getOutputCurrent());

        if (getElevatorZeroed()) {
            if (update_zero) {
                ele_motor.setPosition(0);
                runElevator(0);
                update_zero = false;
            }
        } else if (ele_motor.getPosition().getValueAsDouble() > 0) {
            update_zero = true;
        }

        if (ele_motor.getMotionMagicIsRunning().getValue() == MotionMagicIsRunningValue.Enabled
                && Math.abs(ele_motor.getMotorVoltage().getValueAsDouble() - 0.55) < 0.1
                && atSetpoint()) {
            count++;
            if (count >= 5) stopElevator();
        } else {
            count = 0;
        }

        if (DriverStation.isDisabled()) {
            ele_motor.setControl(new NeutralOut());
        }
    }
}
