package frc.robot.Subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorSetpoints;
import frc.robot.Constants.ElevatorConstants.WristSetpoints;

abstract public class Elevator extends SubsystemBase {

    public double ele_target, wrist_target, ele_setpoint, wrist_setpoint, claw_speed, shooter_speed;

    // An enum to represent different target states for the elevator, containing elevator and wrist setpoints
    enum TargetState {
        kStow(ElevatorSetpoints.kStow, WristSetpoints.kStow), 
        kProcessor(ElevatorSetpoints.kProcessor, WristSetpoints.kProcessor),
        kSource(ElevatorSetpoints.kSource, WristSetpoints.kSource),
        kL1(ElevatorSetpoints.kL1, WristSetpoints.kL1), 
        kL2(ElevatorSetpoints.kL2, WristSetpoints.kL2), 
        kL3(ElevatorSetpoints.kL3, WristSetpoints.kL3), 
        kL4(ElevatorSetpoints.kL4, WristSetpoints.kL4);

        public final double elevator_setpoint, wrist_setpoint;
        private TargetState(double elevator_setpoint, double wrist_setpoint) {
            this.elevator_setpoint = elevator_setpoint;
            this.wrist_setpoint = wrist_setpoint;
        }
    }

    public Elevator() {
        ele_target = getElevatorHeight();
        ele_setpoint = getElevatorHeight();
        wrist_target = getWristAngle();
        wrist_setpoint = getWristAngle();
        claw_speed = 0;
        shooter_speed = 0;
    }

    /**
     * Sets the target state for the elevator and wrist.
     * @param state The desired state.
     */
    public void setState(TargetState state) {
        ele_target = state.elevator_setpoint;
        wrist_target = state.wrist_setpoint;
    }

    /**
     * Sets the target setpoint for internal elevator PID
     * @param setpoint The setpoint in meters. 0 is the lowest elevator height.
     */
    public void setElevatorSetpoint(double setpoint) {
        ele_setpoint = MathUtil.clamp(setpoint, 0, ElevatorConstants.MAX_EXTENSION);
        setElePID(ele_setpoint);
    }

    /**
     * Sets the target setpoint for internal wrist PID
     * @param setpoint The setpoint in meters. 0 is the lowest elevator height.
     */
    public void setWristSetpoint(double setpoint) {
        wrist_setpoint = MathUtil.clamp(setpoint, 0, ElevatorConstants.MAX_EXTENSION);
        setElePID(wrist_setpoint);
    }

    public void changeSetpoint(double setpoint) {
        ele_setpoint += setpoint;
        setElevatorSetpoint(ele_setpoint);
    }

    public void runShooter(double speed) {
        shooter_speed = speed;
        setShooterSpeed(speed);
    }

    // VoltageOut() methods
    public abstract void runElevator(double speed);
    public abstract void runWrist(double speed);
    // PID set methods
    protected abstract void setElePID(double setpoint);
    protected abstract void setWristPID(double setpoint);
    // Spark set methods
    protected abstract void setClawSpeed(double speed);
    protected abstract void setShooterSpeed(double speed);
    // Getters
    public abstract double getElevatorHeight();
    public abstract double getWristAngle();

    @Override
    public void periodic() {

    } 

}
