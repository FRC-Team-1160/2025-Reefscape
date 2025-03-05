package frc.robot.Subsystems.Elevator;

import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.AutoLogOutput;

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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorConfigs;
import frc.robot.Constants.ElevatorConstants.ElevatorMotionMagic;
import frc.robot.Constants.ElevatorConstants.WristConfigs;
import frc.robot.Constants.ElevatorConstants.WristMotionMagic;
import frc.robot.Constants.ElevatorConstants.WristSetpoints;


public class ElevatorRealIO extends Elevator {

    protected TalonFX ele_motor, wrist_motor;

    private SparkMax intake_motor, shooter_motor;

    private DigitalInput elevator_switch, coral_switch;

    private boolean update_zero;

    protected ElevatorRealIO() {
        ele_motor = new TalonFX(PortConstants.ELEVATOR_MOTOR);
        wrist_motor = new TalonFX(PortConstants.WRIST_MOTOR);

        intake_motor = new SparkMax(PortConstants.INTAKE_MOTOR, MotorType.kBrushless);
        shooter_motor = new SparkMax(27, MotorType.kBrushless);

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

        TalonFXConfiguration wrist_configs = new TalonFXConfiguration();
        wrist_configs.Slot0 = new Slot0Configs()
            .withKP(WristConfigs.kP)
            .withKI(WristConfigs.kI)
            .withKD(WristConfigs.kD)
            .withKS(WristConfigs.kS)
            .withKV(WristConfigs.kV)
            .withKA(WristConfigs.kA)
            .withKG(WristConfigs.kG);

        wrist_configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        wrist_configs.Feedback.SensorToMechanismRatio = 72;

        wrist_configs.MotionMagic = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(WristMotionMagic.VELOCITY)
            .withMotionMagicAcceleration(WristMotionMagic.ACCELERATION)
            .withMotionMagicJerk(WristMotionMagic.JERK)
            .withMotionMagicExpo_kV(WristMotionMagic.EXPO_kV)
            .withMotionMagicExpo_kA(WristMotionMagic.EXPO_kA);          

        wrist_motor.getConfigurator().apply(wrist_configs);

        update_zero = true;
        // ele_setpoint = ele_motor_right.getPosition().getValueAsDouble();
        // wrist_setpoint = wrist_motor.getPosition().getValueAsDouble();
    }

    public double runElevator(double speed) {
        ele_motor.setControl(new VoltageOut(speed));
        return speed;
    }

    public double runWrist(double speed) {
        wrist_motor.setControl(new VoltageOut(speed));
        return speed;
    }

    //DON'T ACTIVATE UNTIL FULLY TUNED
    protected void runEleMotionMagic(double setpoint){
        ele_motor.setControl(new MotionMagicVoltage(setpoint));
    }

    //DON'T ACTIVATE UNTIL FULLY TUNED
    protected void runWristMotionMagic(double setpoint) {
        wrist_motor.setControl(new MotionMagicVoltage(setpoint));
    }

    public void changeWristSetpoint(double rate) {
        if (wrist_motor.getMotionMagicIsRunning().getValue() == MotionMagicIsRunningValue.Enabled) {
            wrist_motor.setControl(new MotionMagicVoltage(
                wrist_motor.getClosedLoopReference().getValueAsDouble() + rate * 0.02));
        } else {
            wrist_motor.setControl(new MotionMagicVoltage(
                wrist_motor.getPosition().getValueAsDouble() + rate * 0.02));
        }
    }

    public void runIntake(double speed) {
        intake_motor.set(speed);
    }

    public void runShooter(double speed) {
        SmartDashboard.putNumber("vibe check", speed);
        shooter_motor.set(-speed);
    }

    public double getElevatorHeight() {
        return ele_motor.getPosition().getValueAsDouble();
    }

    public Rotation2d getWristAngle() {
        return Rotation2d.fromRotations(wrist_motor.getPosition().getValueAsDouble());
    }

    public void zeroWrist() {
        System.out.println("YAYAYAY");
        wrist_motor.setPosition(0.195);
    }

    // public Command intakeCoralCmd() {
    //     return Commands.sequence(
    //         new InstantCommand(() -> runShooter(0.25)),
    //         new WaitCommand(0.2),
    //         new WaitUntilCommand(() -> shooter_motor.getOutputCurrent() > 10),
    //         new WaitCommand(1.0)
    //     ).finallyDo(() -> runShooter(0));
    // }

    public Command intakeCoralCmd() {
        return Commands.sequence(
            new InstantCommand(() -> runShooter(0.25)),
            new WaitUntilCommand(this::getCoralStored),
            new WaitCommand(0.2)
        ).finallyDo(() -> runShooter(0));
    }

    public Command intakeCoralSequence() {
        return Commands.sequence(
            new InstantCommand(() -> {
                setState(TargetState.kSource);
                runShooter(0.25);
            }),
            new WaitUntilCommand(this::getCoralStored),
            new WaitCommand(0.3),
            new InstantCommand(() -> {
                runShooter(-0.15);
                setWristSetpoint(WristSetpoints.kReefCoral);
            }),
            new WaitCommand(0.2)
        ).finallyDo(() -> runShooter(0));
    }

    public Command intakeAlgaeCmd() {
        return Commands.sequence(
            new InstantCommand(() -> runIntake(-0.7)),
            new WaitCommand(0.1),
            new WaitUntilCommand(() -> intake_motor.getOutputCurrent() > 11),
            new InstantCommand(() -> runIntake(-0.3)),
            new WaitCommand(1)
        ).finallyDo(() -> runIntake(0));
    }

    public List<TalonFX> getTalons() {
        return Arrays.asList(ele_motor, wrist_motor);
    }

    public boolean getCoralStored() {
        return !coral_switch.get();
    }

    public boolean getElevatorZeroed() {
        return !elevator_switch.get();
    }

    @Override
    public void periodic() {

        if (!elevator_switch.get()) {
            if (update_zero) {
                ele_motor.setPosition(0);
                update_zero = false;
            }
        } else {
            update_zero = true;
        }

        if (ele_motor.getMotionMagicIsRunning().getValue() == MotionMagicIsRunningValue.Enabled
                && Math.abs(ele_motor.getMotorVoltage().getValueAsDouble() - 0.5) < 0.05
                && Math.abs(ele_motor.getClosedLoopError().getValueAsDouble()) < 0.05) {
            ele_motor.setControl(new VoltageOut(0.35));
        } else if (ele_motor.getPosition().getValueAsDouble() < 0) {
            ele_motor.setPosition(0);
        }

        if (DriverStation.isDisabled()) {
            ele_motor.setControl(new NeutralOut());
            wrist_motor.setControl(new NeutralOut());
        }
    }
}
