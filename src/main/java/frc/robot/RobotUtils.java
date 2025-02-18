package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

public final class RobotUtils {
    
    public static double hypot(double a, double b) {
        return Math.sqrt(a * a + b * b);
    }

    public static double hypot_inverse(double c, double a) {
        return Math.sqrt(c * c - a * a);
    }

    public static double clampAbs(double x, double maxAbs) {
        return MathUtil.clamp(x, -maxAbs, maxAbs);
    }

    public static double allianceFlip(double x) {
        if (DriverStation.getAlliance().isEmpty()) return x;
        return DriverStation.getAlliance().get() == Alliance.Red ? FieldConstants.LENGTH - x : x;
    }

}
