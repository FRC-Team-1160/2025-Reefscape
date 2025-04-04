package frc.robot.Subsystems.Elevator;

import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import frc.robot.Robot;
import frc.robot.RobotUtils;
import frc.robot.SubsystemManager;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorSetpoints;

abstract public class Elevator extends SubsystemBase {

    public static final Elevator instance = Robot.isReal() ? new ElevatorRealIO() : new ElevatorSimIO();

    @AutoLogOutput(key = "Elevator/Current State")
    public TargetState m_current_state;

    public enum LEDPattern {
        kBlue(0.85),
        kBlueFlash(-0.09),
        kWhite(0.93),
        kWhiteFlash(-0.05);

        public final double value;
        private LEDPattern(double value) {
            this.value = value;
        }
    }

    // An enum to represent different target states for the elevator, containing elevator and wrist setpoints
    public enum TargetState {
        kStow(ElevatorSetpoints.kStow, null, AlignCommand.kNone), 
        kSource(ElevatorSetpoints.kSource, null, AlignCommand.kSource),
        kL1(ElevatorSetpoints.kL1, null, AlignCommand.kReefCoral), 
        kL2(ElevatorSetpoints.kL2, null, AlignCommand.kReefCoral), 
        kL3(ElevatorSetpoints.kL3, null, AlignCommand.kReefCoral), 
        kL4(ElevatorSetpoints.kL4, null, AlignCommand.kReefCoral),
        kL2Algae(ElevatorSetpoints.kL2Algae, null, AlignCommand.kReefAlgae),
        kL3Algae(ElevatorSetpoints.kL3Algae, null, AlignCommand.kReefAlgae);


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
            kSource(() -> SubsystemManager.instance.commands.alignSource()),
            kReefCoral(() -> SubsystemManager.instance.commands.alignReef()),
            kReefAlgae(() -> SubsystemManager.instance.commands.alignReefAlgae());

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
        return new WaitUntilCommand(() -> m_current_state != state || atSetpoint())
            .beforeStarting(() -> setState(state));
    }

    @AutoLogOutput
    public boolean atSetpoint() {
        return atSetpoint(m_current_state.elevator_setpoint);
    }

    public boolean atSetpoint(double setpoint) {
        return Math.abs(Math.max(0, getElevatorHeight()) - Math.max(0, setpoint)) < 0.03;
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

    // Flywheel set methods
    public abstract void runAlgae(double speed);
    public abstract void runShooter(double speed);

    // Getters
    @AutoLogOutput
    public abstract double getElevatorHeight();

    public Command intakeCoralCmd(boolean exit_early) {
        Command end_sequence = new WaitCommand(0.1)
            .andThen(RobotUtils.onOffCommand(this::runShooter, 0.15)
                .withTimeout(0.2));
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

    public abstract void setLEDs(LEDPattern pattern);

    public abstract List<TalonFX> getTalons();

    @Override
    public void periodic() {

    }

}
