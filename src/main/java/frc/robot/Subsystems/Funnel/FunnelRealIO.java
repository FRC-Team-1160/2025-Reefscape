package frc.robot.Subsystems.Funnel;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PortConstants;

public class FunnelRealIO extends Funnel {

    private Servo servo_left, servo_right;

    protected FunnelRealIO() {
        servo_left = new Servo(1);
        servo_right = new Servo(0);
    }

    protected void setLeftServo(double setpoint) {
        servo_left.setAngle(setpoint);
    }

    protected void setRightServo(double setpoint) {
        servo_right.setAngle(setpoint);
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}
