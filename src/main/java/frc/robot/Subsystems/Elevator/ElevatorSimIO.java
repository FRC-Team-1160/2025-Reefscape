package frc.robot.Subsystems.Elevator;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RobotConstants;

public class ElevatorSimIO extends Elevator {

    private double ele_pos, ele_speed, ele_setpoint;

    private boolean ele_pid;

    protected ElevatorSimIO() {
        ele_pid = false;
    }

    public double runElevator(double speed) {
        ele_pid = false;
        return ele_speed = speed;
    }

    public double runWrist(double speed) { return speed; }

    public Command setWristCmd(boolean up) { return Commands.none(); }

    protected void runEleMotionMagic(double setpoint) {
        ele_pid = true;
        if (ele_setpoint == setpoint) return;
        ele_setpoint = setpoint;
        if (Math.abs(ele_pos - setpoint) > 0.1) {
            ele_speed = 4 * (setpoint > ele_pos ? 1 : -1);
        } else {
            ele_speed = 0;
        }
    }

    public void stopElevator() {
        ele_speed = 0;
    }

    public void runAlgae(double speed) {}

    public void runShooter(double speed) {}

    public double getElevatorHeight() { return ele_pos; }

    public boolean getCoralStored() { return true; }

    public boolean getElevatorZeroed() { return ele_pos == 0; }

    public boolean atSetpoint() {
        return Math.abs(ele_setpoint - ele_pos) < 0.02;
    }

    @Override
    public void periodic() {
        super.periodic();
        ele_pos += ele_speed * RobotConstants.LOOP_TIME_SECONDS;

        if (ele_pos >= ElevatorConstants.MAX_EXTENSION) {
            ele_pos = ElevatorConstants.MAX_EXTENSION;
            if (ele_speed > 0) ele_speed = 0;
        }
        if (ele_pos <= 0) {
            ele_pos = 0;
            if (ele_speed < 0) ele_speed = 0;
        }

        if (ele_pid && atTarget()) ele_speed = 0;
    }
}