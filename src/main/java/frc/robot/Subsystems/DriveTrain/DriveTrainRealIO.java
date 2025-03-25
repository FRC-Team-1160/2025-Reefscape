package frc.robot.Subsystems.DriveTrain;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.hardware.TalonFX;
import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class DriveTrainRealIO extends DriveTrain {

    private AHRS gyro;

    protected DriveTrainRealIO(){
        gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
    }

    public SwerveModule initializeModule(int drive_port, int steer_port, int sensor_port, Translation2d offset) {
        return new SwerveModuleRealIO(drive_port, steer_port, sensor_port, offset);
    }

    public Rotation2d getGyroAngle() {
        // Gyro reports CW positive, negate to return CCW positive
        if (gyro != null) return Rotation2d.fromDegrees(-gyro.getAngle()); 
        return new Rotation2d();
    }

    public double getGyroRate(){
        if (gyro != null) return -gyro.getRate(); 
        return 0.0;
    }

    public void setGyroAngle(double angle){
        if (gyro == null) return;        
        gyro.zeroYaw();
        gyro.setAngleAdjustment(-angle);
    }

    public void resetGyroAngle() {
        if (gyro == null) return;
        gyro.zeroYaw();
        gyro.setAngleAdjustment(0);
    }

    public List<TalonFX> getTalons() {
        return Arrays.stream(modules).map(module -> module.getTalons()).flatMap(List::stream).toList();
    }

    @Override
    public void periodic() {
        // The child periodic() method overrides the parent class periodic(), which has to be explicitly called
        super.periodic();
    }
}
