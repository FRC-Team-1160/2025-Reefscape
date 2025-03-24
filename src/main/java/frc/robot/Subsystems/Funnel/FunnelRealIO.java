package frc.robot.Subsystems.Funnel;

import edu.wpi.first.wpilibj.PWM;

public class FunnelRealIO extends Funnel {

    private PWM servo_left, servo_right;

    protected FunnelRealIO() {
        servo_left = new PWM(1);
        servo_right = new PWM(0);
    }

    protected void setLeftServo(double speed) {
        servo_left.setSpeed(speed);
    }

    protected void setRightServo(double speed) {
        servo_right.setSpeed(speed);
    }

    public double getLeftSpeed() {
        return servo_left.getSpeed();
    }

    public double getRightSpeed() {
        return servo_right.getSpeed();
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}
