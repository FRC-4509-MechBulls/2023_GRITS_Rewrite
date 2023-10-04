package frc.robot;

public class MBUtils {

    public static double lerp(double value1, double value2, double t) {
        return (1 - t) * value1 + t * value2;
    }
}
