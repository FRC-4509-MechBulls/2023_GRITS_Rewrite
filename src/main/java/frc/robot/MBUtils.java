package frc.robot;

public class MBUtils {

    public static double lerp(double value1, double value2, double t) { //linear interpolation!! (for shrimp)
        return (1 - t) * value1 + t * value2;
    }

    public static double clamp(double input, double absMax){
        absMax = Math.abs(absMax);

        if(input>absMax)
            input = absMax;
        if(input<-absMax)
            input = -absMax;

        return input;
    }

    public static double angleDiffDeg(double ang1, double ang2){
        if(Math.abs(ang1-ang2)>180){
            if(ang1>ang2)
                return -ang1+ang2+360;
            else
                return -ang1+ang2-360;
        }
        return ang2-ang1;
    }

    public static double clamp(double input, double min, double max){
        if(input>max) input = max;
        if(input<min) input = min;
        return input;
    }

    public static double interpolate(double[] xValues, double[] yValues, double x) {
        // Ensure the arrays have the same length
        if (xValues.length != yValues.length) {
            throw new IllegalArgumentException("xValues and yValues must have the same length.");
        }

        int indexBelow = -1;
        int indexAbove = -1;

        // Find the two indices where x lies in between
        for (int i = 0; i < xValues.length - 1; i++) {
            if (x >= xValues[i] && x <= xValues[i + 1]) {
                indexBelow = i;
                indexAbove = i + 1;
                break;
            }
        }

        // If indices found, interpolate. Otherwise, return -1 (or any other error value).
        if (indexBelow != -1) {
            double x1 = xValues[indexBelow];
            double x2 = xValues[indexAbove];
            double y1 = yValues[indexBelow];
            double y2 = yValues[indexAbove];

            return y1 + ((x - x1) / (x2 - x1)) * (y2 - y1);
        } else {
            // You can throw an exception or return a default/error value.
            // In this example, we return 0 to indicate an error.
            return 0;
        }
    }


}
