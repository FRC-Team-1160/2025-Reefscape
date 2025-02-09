package frc.robot.Subsystems.Elevator;

public class ElevatorSimIO extends Elevator {

    public double ele_pos, wrist_pos, left_claw_speed, right_claw_speed;

    public ElevatorSimIO () {}

    protected void setEleVoltage(double volts) {}
    protected void setWristVoltage(double volts) {}

    protected void setElePID(double setpoint) {}
    protected void setWristPID(double setpoint) {}

    protected void setLeftClawSpeed(double speed) {}
    protected void setRightClawSpeed(double speed) {}
    protected void setShooterSpeed(double speed) {}

}