package frc.robot.Subsystems.DriveTrain;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.DriveMotorConfigs;
import frc.robot.Constants.SwerveConstants.SteerMotorConfigs;


public class SwerveModuleRealIO extends SwerveModule {

    private TalonFX steer_motor, drive_motor;

    private CANcoder steer_sensor;

    protected SwerveModuleRealIO(int drive_port, int steer_port, int sensor_port) {
        drive_motor = new TalonFX(drive_port, "CANivore");
        steer_motor = new TalonFX(steer_port, "CANivore");
        steer_sensor = new CANcoder(sensor_port, "CANivore");
        
        TalonFXConfiguration drive_configs = new TalonFXConfiguration();

        drive_configs.Slot0 = new Slot0Configs()
            .withKP(DriveMotorConfigs.kP)
            .withKI(DriveMotorConfigs.kI)
            .withKD(DriveMotorConfigs.kD)
            .withKS(DriveMotorConfigs.kS)
            .withKV(DriveMotorConfigs.kV)
            .withKA(DriveMotorConfigs.kA)
            .withKG(DriveMotorConfigs.kG);

        drive_configs.Feedback.SensorToMechanismRatio = 5.01; // 5.01;

        drive_motor.getConfigurator().apply(drive_configs);

        TalonFXConfiguration steer_configs = new TalonFXConfiguration();

        steer_configs.Slot0 = new Slot0Configs()
            .withKP(SteerMotorConfigs.kP)
            .withKI(SteerMotorConfigs.kI)
            .withKD(SteerMotorConfigs.kD)
            .withKS(SteerMotorConfigs.kS)
            .withKV(SteerMotorConfigs.kV)
            .withKA(SteerMotorConfigs.kA)
            .withKG(SteerMotorConfigs.kG);

        steer_configs.Feedback.FeedbackRemoteSensorID = sensor_port;
        steer_configs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        steer_configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        steer_configs.Voltage.PeakForwardVoltage = 3.5;
        steer_configs.Voltage.PeakReverseVoltage = -3.5;

        steer_configs.ClosedLoopGeneral.ContinuousWrap = true;

        steer_motor.getConfigurator().apply(steer_configs);

    }

    public double getSpeed() {
        //getRotorVelocity() returns StatusSignal<AngularVelocity> with base unit rps
        return drive_motor.getRotorVelocity().getValueAsDouble()
            * SwerveConstants.WHEEL_DIAMETER / SwerveConstants.GEAR_RATIO;
    }

    public double getPosition() {
        //getRotorPosition() returns StatusSignal<Angle> with base unit rotations
        return drive_motor.getRotorPosition().getValueAsDouble() 
            * SwerveConstants.WHEEL_DIAMETER / SwerveConstants.GEAR_RATIO;
    }

    public Rotation2d getAngle() {
        //getPosition() returns StatusSignal<Angle> with base unit rotations
        double a = steer_sensor.getAbsolutePosition().getValueAsDouble();
        a = MathUtil.inputModulus(a, -0.5, 0.5);
        return Rotation2d.fromRotations(a);
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getSpeed(), getAngle());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getPosition(), getAngle());
    }

    protected void setSpeed(double speed) {
        drive_motor.setControl(new VelocityVoltage(speed / SwerveConstants.WHEEL_DIAMETER));
    }

    protected void setAngle(Rotation2d angle) {
        steer_motor.setControl(new PositionVoltage(angle.getRotations())); 
    }

    public List<TalonFX> getTalons() {
        return Arrays.asList(drive_motor, steer_motor);
    }

}
