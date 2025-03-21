package frc.robot.Subsystems.Elevator;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.lang.annotation.Target;
import java.util.HashMap;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.RobotUtils;
import frc.robot.SubsystemManager;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorSetpoints;

abstract public class Elevator extends SubsystemBase {

    public static final Elevator instance = Robot.isReal() ? new ElevatorRealIO() : new ElevatorSimIO();

    public TargetState m_current_state;

    // An enum to represent different target states for the elevator, containing elevator and wrist setpoints
    public enum TargetState {
        kStow(ElevatorSetpoints.kStow, true, AlignCommand.kNone), 
        kSource(ElevatorSetpoints.kSource, null, AlignCommand.kSource),
        kL1(ElevatorSetpoints.kL1, false, AlignCommand.kReefCoral), 
        kL2(ElevatorSetpoints.kL2, false, AlignCommand.kReefCoral), 
        kL3(ElevatorSetpoints.kL3, false, AlignCommand.kReefCoral), 
        kL4(ElevatorSetpoints.kL4, false, AlignCommand.kReefCoral),
        kL2Algae(ElevatorSetpoints.kL2Algae, true, AlignCommand.kReefAlgae),
        kL3Algae(ElevatorSetpoints.kL3Algae, true, AlignCommand.kReefAlgae);


        public final Double elevator_setpoint;
        public final Boolean wrist_setpoint;
        public final AlignCommand target_position;

        private TargetState(Double elevator_setpoint, Boolean wrist_setpoint, AlignCommand target_position) {
            this.elevator_setpoint = elevator_setpoint;
            this.wrist_setpoint = wrist_setpoint;
            this.target_position = target_position;
        }

        public enum AlignCommand {
            kNone(() -> Commands.none()),
            kProcessor(() -> Commands.none()),
            kSource(() -> SubsystemManager.instance.commands.alignSource(false)),
            kReefCoral(() -> SubsystemManager.instance.commands.alignReef()),
            kReefCoralL4(() -> SubsystemManager.instance.commands.alignReefClose()),
            kReefAlgae(() -> SubsystemManager.instance.commands.alignReefAlgae()),
            kGround(() -> SubsystemManager.instance.commands.trackAlgae());

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
        setElevatorSetpoint(state.elevator_setpoint);
        setWrist(state.wrist_setpoint);
    }

    /**
     * Sets the target setpoint for internal elevator PID
     * @param setpoint The setpoint in meters. 0 is the lowest elevator height.
     */
    public void setElevatorSetpoint(double setpoint) {
        runEleMotionMagic(MathUtil.clamp(setpoint, ElevatorConstants.MIN_EXTENSION, ElevatorConstants.MAX_EXTENSION));
    }

    public Command setStateCmd(TargetState state) {
        return new WaitUntilCommand(() -> m_current_state != state || atTarget())
            .beforeStarting(() -> setState(state));
    }

    @AutoLogOutput
    public boolean atTarget() {
        return atTarget(m_current_state);
    }

    public boolean atTarget(TargetState state) {
        return Math.abs(getElevatorHeight() - state.elevator_setpoint) < 0.02;
    }

    // VoltageOut() methods
    @AutoLogOutput
    public abstract double runElevator(double speed);
    @AutoLogOutput
    public abstract double runWrist(double speed);
    // PID set methods
    protected abstract void runEleMotionMagic(double setpoint);

    public abstract void stopElevator();

    public void setWrist(Boolean up) {
        if (up != null) CommandScheduler.getInstance().schedule(setWristCmd(up));
    }
    public abstract Command setWristCmd(boolean up);

    // Spark set methods
    public abstract void runAlgae(double speed);
    public abstract void runShooter(double speed);
    // Getters
    @AutoLogOutput
    public abstract double getElevatorHeight();

    public Command intakeCoralCmd(boolean exit_early) {
        Command end_sequence = new WaitCommand(0.2)
            .andThen(RobotUtils.onOffCommand(this::runShooter, 0.2))
            .withTimeout(0.2);
        return RobotUtils.onOffCommand(this::runShooter, 0.2)
                .until(this::getCoralStored)
                .andThen(exit_early
                     ? new InstantCommand(() -> CommandScheduler.getInstance().schedule(end_sequence))
                     : end_sequence
                ).finallyDo(() -> runShooter(0));
    }

    public Command intakeCoralCmd() {
        return intakeCoralCmd(false);
    }

    @AutoLogOutput
    public abstract boolean getCoralStored();
    @AutoLogOutput
    public abstract boolean getElevatorZeroed();
    @AutoLogOutput
    public abstract boolean atSetpoint();

    @Override
    public void periodic() {
        Logger.recordOutput("Elevator/State", m_current_state.toString());

    }

}
