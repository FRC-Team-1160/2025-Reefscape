package frc.robot.Subsystems.Elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.RobotConstants;

public class ElevatorSimIO extends Elevator {

    private double ele_pos, wrist_pos, ele_speed, wrist_speed, claw_speed, shooter_speed;

    private boolean ele_pid, wrist_pid;

    public ElevatorSimIO() {
        ele_pid = false;
        wrist_pid = false;
    }

    public void runElevator(double speed) {
        ele_speed = speed;
        ele_pid = false;
    }

    public void runWrist(double speed) {
        wrist_speed = speed;
        wrist_pid = false;
    }

    protected void setElePID(double setpoint) {
        ele_pid = true;
        if (Math.abs(ele_pos - setpoint) > 0.05) {
            ele_speed = 1.5 * (setpoint > ele_pos ? 1 : -1);
        } else {
            ele_speed = 0;
        }
    }

    protected void setWristPID(double setpoint) {
        wrist_pid = true;
        if (Math.abs(wrist_pos - setpoint) > 0.05) {
            wrist_speed = (setpoint > wrist_pos) ? 1 : -1;
            System.out.println("wrist_speed");
            System.out.println(wrist_speed);

        } else {
            wrist_speed = 0;
        }
    }

    protected void setClawSpeed(double speed) {
        claw_speed = speed;
    }

    protected void setShooterSpeed(double speed) {
        shooter_speed = speed;
    }

    public double getElevatorHeight() { 
        return ele_pos; 
    }

    public Rotation2d getWristAngle() {
        return Rotation2d.fromRadians(wrist_pos);
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

        if (ele_pid && (Math.abs(ele_pos - ele_setpoint) <= 0.05)) ele_speed = 0;

        wrist_pos += wrist_speed * RobotConstants.LOOP_TIME_SECONDS;

        if (wrist_pos >= ElevatorConstants.MAX_WRIST_ANGLE) {
            wrist_pos = ElevatorConstants.MAX_WRIST_ANGLE;
            if (wrist_speed > 0) wrist_speed = 0;
        }
        if (wrist_pos <= ElevatorConstants.MIN_WRIST_ANGLE) {
            wrist_pos = 0;
            if (wrist_speed < 0) wrist_speed = 0;
        }

        if (wrist_pid && (Math.abs(wrist_pos - wrist_setpoint) <= 0.05)) wrist_speed = 0;

    }
}