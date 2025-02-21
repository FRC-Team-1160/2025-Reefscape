package frc.robot.Subsystems.Funnel;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants.PortConstants;

public class FunnelRealIO extends Funnel {

    public Servo servo_left, servo_right;

    public FunnelRealIO() {
        servo_left = new Servo(PortConstants.SERVO_LEFT);
        servo_right = new Servo(PortConstants.SERVO_RIGHT);
    }

    protected void setLeftServo(double setpoint) {
        servo_left.set(setpoint);
    }

    protected void setRightServo(double setpoint) {
        servo_right.set(setpoint);
    }
}
