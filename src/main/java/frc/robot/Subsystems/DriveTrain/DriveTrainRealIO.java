package frc.robot.Subsystems.DriveTrain;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.hardware.TalonFX;
import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;

public class DriveTrainRealIO extends DriveTrain {

    private AHRS gyro;

    public DriveTrainRealIO(){
        gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
    }

    public SwerveModule initializeModule(int drive_port, int steer_port, int sensor_port){
        return new SwerveModuleRealIO(drive_port, steer_port, sensor_port);
    }

    public Rotation2d getGyroAngle() {
        // Gyro reports CW positive, negate to return CCW positive
        if (gyro != null) return Rotation2d.fromDegrees(-gyro.getAngle()); 
        return new Rotation2d();
    }

    public void resetGyroAngle() {
        if (gyro != null) gyro.zeroYaw();
    }

    public List<TalonFX> getTalons() {
        List<TalonFX> talons = new ArrayList<TalonFX>();
        for (SwerveModule module : modules) {
            talons.addAll(((SwerveModuleRealIO)module).getTalons());
        }
        return talons;
    }

    @Override
    public void periodic() {
        // The child periodic() method overrides the parent class periodic(), which has to be explicitly called
        super.periodic();
    }
}
