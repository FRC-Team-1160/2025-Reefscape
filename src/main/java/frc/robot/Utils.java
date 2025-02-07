package frc.robot;

public final class Utils {
    
    public static double hypot(double a, double b) {
        return Math.sqrt(a*a + b*b);
    }

    public static double hypot_inverse(double c, double a) {
        return Math.sqrt(c*c - a*a);
    }

    public static double clamp(double x, double min, double max) {
        return Math.min(Math.max(x, min), max);
    }

    public static double clamp(double x, double maxAbs) {
        return clamp(x, -maxAbs, maxAbs);
    }

    public static double threshold(double x, double t) {
        return Math.abs(x) > t ? x : 0;
    }

}
