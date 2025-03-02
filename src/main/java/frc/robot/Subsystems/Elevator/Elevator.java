package frc.robot.Subsystems.Elevator;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.units.measure.MutVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.SubsystemManager;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorSetpoints;
import frc.robot.Constants.ElevatorConstants.WristSetpoints;

abstract public class Elevator extends SubsystemBase {

    public static final Elevator instance = Robot.isReal() ? new ElevatorRealIO() : new ElevatorSimIO();

    public TargetState m_current_state;

      // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private static final MutVoltage m_appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private static final MutDistance m_distance = Meters.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private static final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

    public static final SysIdRoutine id_routine_up = new SysIdRoutine(
        new SysIdRoutine.Config(
            new MutVelocity<VoltageUnit>(0.5, 0.5, Units.Volt.per(Units.Second)),
            new MutVoltage(0.5, 0.5, Units.Volt),
            new MutTime(6, 6, Units.Second)
        ), 
        new SysIdRoutine.Mechanism(
            voltage -> instance.runElevator(voltage.baseUnitMagnitude()), 
            log -> {
                // Record a frame for the left motors.  Since these share an encoder, we consider
                // the entire group to be one motor.
                log.motor("drive-left")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            ((ElevatorRealIO)Elevator.instance).ele_motor.getMotorVoltage().getValueAsDouble(), Volts))
                    .linearPosition(m_distance.mut_replace(((ElevatorRealIO)Elevator.instance).ele_motor.getPosition().getValueAsDouble(), Meters))
                    .linearVelocity(
                        m_velocity.mut_replace(((ElevatorRealIO)Elevator.instance).ele_motor.getVelocity().getValueAsDouble(), MetersPerSecond));}, 
            instance));
    
    public static final SysIdRoutine id_routine_down = new SysIdRoutine(
        new SysIdRoutine.Config(
            new MutVelocity<VoltageUnit>(0.3, 0.3, Units.Volt.per(Units.Second)),
            new MutVoltage(0.3, 0.3, Units.Volt),
            new MutTime(6, 6, Units.Second)
        ), 
        new SysIdRoutine.Mechanism(
            voltage -> instance.runElevator(voltage.baseUnitMagnitude()), 
            state -> Logger.recordOutput("Elevator/SysIdState", state.toString()), 
            instance));

    // An enum to represent different target states for the elevator, containing elevator and wrist setpoints
    public enum TargetState {
        kStow(ElevatorSetpoints.kStow, WristSetpoints.kStow, AlignCommand.kNone), 
        kProcessor(ElevatorSetpoints.kProcessor, WristSetpoints.kProcessor, AlignCommand.kProcessor),
        kSource(ElevatorSetpoints.kSource, WristSetpoints.kSource, AlignCommand.kSource),
        kIntake(ElevatorSetpoints.kIntake, WristSetpoints.kIntake, AlignCommand.kGround),
        kIntakePrepare(ElevatorSetpoints.kIntakePrepare, WristSetpoints.kIntakePrepare, AlignCommand.kGround),
        kBarge(ElevatorSetpoints.kBarge, WristSetpoints.kBarge, AlignCommand.kNone),
        kL1(ElevatorSetpoints.kL1, WristSetpoints.kReefCoral, AlignCommand.kReefCoral), 
        kL2(ElevatorSetpoints.kL2, WristSetpoints.kReefCoral, AlignCommand.kReefCoral), 
        kL3(ElevatorSetpoints.kL3, WristSetpoints.kReefCoral, AlignCommand.kReefCoral), 
        kL4(ElevatorSetpoints.kL4, WristSetpoints.kReefCoral, AlignCommand.kReefCoral),
        kL2Algae(ElevatorSetpoints.kL2Algae, WristSetpoints.kReefAlgae, AlignCommand.kReefAlgae),
        kL3Algae(ElevatorSetpoints.kL3Algae, WristSetpoints.kReefAlgae, AlignCommand.kReefAlgae);


        public final Double elevator_setpoint, wrist_setpoint;
        public final AlignCommand target_position;

        private TargetState(Double elevator_setpoint, Double wrist_setpoint, AlignCommand target_position) {
            this.elevator_setpoint = elevator_setpoint;
            this.wrist_setpoint = wrist_setpoint;
            this.target_position = target_position;
        }

        public enum AlignCommand {
            kNone(() -> Commands.none()),
            kProcessor(() -> SubsystemManager.instance.commands.alignProcessor(false)),
            kSource(() -> SubsystemManager.instance.commands.alignSource(false)),
            kReefCoral(() -> SubsystemManager.instance.commands.alignReef(false)),
            kReefAlgae(() -> Commands.none()),
            kGround(() -> Commands.none());

            public final Supplier<Command> command_supplier;

            private AlignCommand(Supplier<Command> command_supplier) {
                this.command_supplier = command_supplier;
            }
        }
    }

    protected Elevator() {
        m_current_state = TargetState.kStow;
    }

    /**
     * Sets the target state for the elevator and wrist.
     * @param state The desired state.
     */
    public void setState(TargetState state) {
        if (state == null) return;
        m_current_state = state;
        SmartDashboard.putString("Elevator State", m_current_state.toString());
        setElevatorSetpoint(state.elevator_setpoint);
        if (state.wrist_setpoint != null) setWristSetpoint(state.wrist_setpoint);
    }

    /**
     * Sets the target setpoint for internal elevator PID
     * @param setpoint The setpoint in meters. 0 is the lowest elevator height.
     */
    public void setElevatorSetpoint(double setpoint) {
        runEleMotionMagic(MathUtil.clamp(setpoint, 0, ElevatorConstants.MAX_EXTENSION));
    }

    /**
     * Sets the target setpoint for internal wrist PID
     * @param setpoint The setpoint in meters. 0 is the lowest elevator height.
     */
    public void setWristSetpoint(double setpoint) {
        runWristMotionMagic(MathUtil.clamp(
                setpoint, 
                ElevatorConstants.MIN_WRIST_ANGLE, 
                ElevatorConstants.MAX_WRIST_ANGLE));
    }

    // VoltageOut() methods
    public abstract void runElevator(double speed);
    public abstract void runWrist(double speed);
    // PID set methods
    protected abstract void runEleMotionMagic(double setpoint);
    protected abstract void runWristMotionMagic(double setpoint);
    // Spark set methods
    public abstract void runIntake(double speed);
    public abstract void runShooter(double speed);
    // Getters
    public abstract double getElevatorHeight();
    public abstract Rotation2d getWristAngle();

    public abstract Command intakeCoralCmd();
    public abstract Command intakeAlgaeCmd();

    public abstract void zeroWrist();

    @Override
    public void periodic() {

    }

}
