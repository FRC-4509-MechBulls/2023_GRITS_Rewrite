package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

public class ArmCommands {

    public static Command placeConeL3Example(Arm arm){
        double[] x = { 0.19, 0.64, 1.19}; // x values
        double[] y = { 0.03, 0.91, 0.79 }; // y values

        SplineInterpolator interpolator = new SplineInterpolator();
        PolynomialSplineFunction spline = interpolator.interpolate(x, y);

        ArmFollowSplineTimed command = new ArmFollowSplineTimed(arm, spline, false, 1);

        return command;
    }
}