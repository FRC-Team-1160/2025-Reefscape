package frc.robot.Subsystems.Elevator;

import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotionMagicIsRunningValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotUtils;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorConfigs;
import frc.robot.Constants.ElevatorConstants.ElevatorMotionMagic;
import frc.robot.Constants.ElevatorConstants.WristConfigs;
import frc.robot.Constants.ElevatorConstants.WristMotionMagic;


public class ElevatorRealIO extends Elevator {

    protected TalonFX ele_motor;

    private SparkMax algae_motor, wrist_motor, shooter_motor;

    private DigitalInput elevator_switch, coral_switch;

    private boolean update_zero;

    private double count;

    protected ElevatorRealIO() {
        ele_motor = new TalonFX(PortConstants.ELEVATOR_MOTOR, "CANivore");
        
        wrist_motor = new SparkMax(PortConstants.WRIST_MOTOR, MotorType.kBrushless);
        algae_motor = new SparkMax(PortConstants.INTAKE_MOTOR, MotorType.kBrushless);
        shooter_motor = new SparkMax(PortConstants.SHOOTER_MOTOR, MotorType.kBrushless);

        elevator_switch = new DigitalInput(0);
        coral_switch = new DigitalInput(1);

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
        wrist_motor.setVoltage(-speed);
        return speed;
    }

    public Command setWristCmd(boolean up) {
        return RobotUtils.onOffCommand(this::runWrist, up ? 2 : -1.5, up ? 0 : 0).withTimeout(0.5);
    }

    protected void runEleMotionMagic(double setpoint){
        ele_motor.setControl(new MotionMagicVoltage(setpoint));
    }

    public void stopElevator(){
        ele_motor.setControl(new VoltageOut(0.35));
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
                ele_motor.setControl(new VoltageOut(0));
                update_zero = false;
            }
        } else {
            if (ele_motor.getPosition().getValueAsDouble() > 0) {
                update_zero = true;
            }
        }

        if (ele_motor.getMotionMagicIsRunning().getValue() == MotionMagicIsRunningValue.Enabled
                && Math.abs(ele_motor.getMotorVoltage().getValueAsDouble() - 0.48) < 0.07
                && Math.abs(ele_motor.getClosedLoopError().getValueAsDouble()) < 0.04) {
            count++;
            if (count >= 5) ele_motor.setControl(new VoltageOut(0.35));
        } else {
            count = 0;
        }

        if (DriverStation.isDisabled()) {
            ele_motor.setControl(new NeutralOut());
        }
    }
}
