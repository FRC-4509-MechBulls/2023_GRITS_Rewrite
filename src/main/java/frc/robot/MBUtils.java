package frc.robot;

public class MBUtils {

    public static double lerp(double value1, double value2, double t) { //linear interpolation!! (for shrimp)
        return (1 - t) * value1 + t * value2;
    }
}
