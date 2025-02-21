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

    public static boolean isRedAlliance() {
        if (DriverStation.getAlliance().isEmpty()) return false;
        return DriverStation.getAlliance().get() == Alliance.Red;
    }

    public static double allianceFlipX(double x) {
        return isRedAlliance() ? FieldConstants.LENGTH - x : x;
    }

    public static double allianceFlipY(double y) {
        return isRedAlliance() ? FieldConstants.WIDTH - y : y;
    }

    public static double allianceNegate(double x) {
        return isRedAlliance() ? -x : x;
    }

}
