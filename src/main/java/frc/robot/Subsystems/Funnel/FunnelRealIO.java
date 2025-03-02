package frc.robot.Subsystems.Funnel;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PortConstants;

public class FunnelRealIO extends Funnel {

    public Servo servo_left, servo_right;

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
        SmartDashboard.putNumber("Servo position left", servo_right.getPosition());
        SmartDashboard.putNumber("Servo position right", servo_right.getPosition());
    }
}
