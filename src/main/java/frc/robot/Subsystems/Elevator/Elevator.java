package frc.robot.Subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorSetpoints;
import frc.robot.Constants.ElevatorConstants.WristSetpoints;

abstract public class Elevator extends SubsystemBase {

    public static final Elevator instance = Robot.isReal() ? new ElevatorRealIO() : new ElevatorSimIO();

    public TargetState m_current_state;

    // An enum to represent different target states for the elevator, containing elevator and wrist setpoints
    public enum TargetState {
        kStow(ElevatorSetpoints.kStow, WristSetpoints.kStow, AlignTarget.kNone), 
        kProcessor(ElevatorSetpoints.kProcessor, WristSetpoints.kProcessor, AlignTarget.kProcessor),
        kSource(ElevatorSetpoints.kSource, WristSetpoints.kSource, AlignTarget.kSource),
        kIntake(ElevatorSetpoints.kIntake, WristSetpoints.kIntake, AlignTarget.kGround),
        kIntakePrepare(ElevatorSetpoints.kIntakePrepare, WristSetpoints.kIntakePrepare, AlignTarget.kGround),
        kBarge(ElevatorSetpoints.kBarge, WristSetpoints.kBarge, AlignTarget.kNone),
        kL1(ElevatorSetpoints.kL1, WristSetpoints.kL1, AlignTarget.kCoral), 
        kL2(ElevatorSetpoints.kL2, WristSetpoints.kL2, AlignTarget.kCoral), 
        kL3(ElevatorSetpoints.kL3, WristSetpoints.kL3, AlignTarget.kCoral), 
        kL4(ElevatorSetpoints.kL4, WristSetpoints.kL4, AlignTarget.kCoral),
        kL2Algae(ElevatorSetpoints.kL2Algae, WristSetpoints.kL2Algae, AlignTarget.kAlgae),
        kL3Algae(ElevatorSetpoints.kL3Algae, WristSetpoints.kL3Algae, AlignTarget.kAlgae);


        public final double elevator_setpoint, wrist_setpoint;
        public final AlignTarget target_position;

        private TargetState(double elevator_setpoint, double wrist_setpoint, AlignTarget target_position) {
            this.elevator_setpoint = elevator_setpoint;
            this.wrist_setpoint = wrist_setpoint;
            this.target_position = target_position;
        }

        public enum AlignTarget {
            kNone,
            kProcessor,
            kSource,
            kCoral,
            kAlgae,
            kGround
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
        SmartDashboard.putString("Elevator State", state.toString());
        m_current_state = state;
        setElevatorSetpoint(state.elevator_setpoint);
        setWristSetpoint(state.wrist_setpoint);
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

    @Override
    public void periodic() {

    }

}
