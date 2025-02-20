package frc.robot.Subsystems.Elevator;

public class ElevatorSimIO extends Elevator {

    public double ele_pos, wrist_pos, left_claw_speed, right_claw_speed;

    public ElevatorSimIO () {}

    public void runElevator(double speed) {}
    public void runWrist(double speed) {}

    protected void setElePID(double setpoint) {}
    protected void setWristPID(double setpoint) {}

    protected void setClawSpeed(double speed) {}
    protected void setShooterSpeed(double speed) {}

    public double getElevatorHeight() { return 0; }
    public double getWristAngle() { return 0; }
}