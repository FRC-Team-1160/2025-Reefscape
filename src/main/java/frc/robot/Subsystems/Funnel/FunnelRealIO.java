package frc.robot.Subsystems.Funnel;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PortConstants;

public class FunnelRealIO extends Funnel {

    public Servo servo_left, servo_right;

    protected FunnelRealIO() {
        servo_left = new Servo(5);
        servo_right = new Servo(6);
    }

    protected void setLeftServo(double setpoint) {
        // servo_left.set(setpoint);
    }

    protected void setRightServo(double setpoint) {
        // servo_right.set(setpoint);
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}
