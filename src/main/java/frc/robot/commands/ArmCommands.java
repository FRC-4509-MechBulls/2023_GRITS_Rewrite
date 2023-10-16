package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Arm.ArmFollowSplineTimed;
import frc.robot.commands.Arm.ArmToPointLinearTimed;
import frc.robot.subsystems.arm.Arm;
import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

public class ArmCommands {

    public static Command placeConeL3Example(Arm arm){
        double[] x = { 0.19, 0.64, 1.19}; // x values
        double[] y = { 0.03, 0.91, 0.79 }; // y values
        //   double[] x = { 0.19, 0.67, 1.19}; // x values
        //   double[] y = { 0.03, 0.75, 0.79 }; // y values

        //0.16, 0.03
        //0.67,0.57
        //1.16,0.68


        SplineInterpolator interpolator = new SplineInterpolator();
        PolynomialSplineFunction spline = interpolator.interpolate(x, y);


        ArmFollowSplineTimed command = new ArmFollowSplineTimed(arm, spline, false, 1);

        return command;
    }

    public static Command placeConeL2Example(Arm arm){
        double[] x = { 0.19, 0.69, .86}; // x values
        double[] y = { 0.03, 0.48, 0.48 }; // y values
        //   double[] x = { 0.19, 0.67, 1.19}; // x values
        //   double[] y = { 0.03, 0.75, 0.79 }; // y values

        //0.16, 0.03
        //0.67,0.57
        //1.16,0.68


        SplineInterpolator interpolator = new SplineInterpolator();
        PolynomialSplineFunction spline = interpolator.interpolate(x, y);


        ArmFollowSplineTimed command = new ArmFollowSplineTimed(arm, spline, false, 0.5);

        return command;
    }

    public static Command retractFromConeL2(Arm arm){
        double[] x = { 0.19, 0.69, .86}; // x values
        double[] y = { 0.03, 0.48, 0.48 }; // y values
        //   double[] x = { 0.19, 0.67, 1.19}; // x values
        //   double[] y = { 0.03, 0.75, 0.79 }; // y values

        //0.16, 0.03
        //0.67,0.57
        //1.16,0.68


        SplineInterpolator interpolator = new SplineInterpolator();
        PolynomialSplineFunction spline = interpolator.interpolate(x, y);


        ArmFollowSplineTimed command = new ArmFollowSplineTimed(arm, spline, true, 0.5);

        return command;
    }


    public static Command retractFromConeL3(Arm arm){
        //  double[] x = { 0.19, 0.64, 1.19}; // x values
        // double[] y = { 0.03, 0.91, 0.79 }; // y values
  //      double[] x = { 0.19, 0.74, 1.19}; // x values
  //      double[] y = { 0.03, 1, 0.79 }; // y values

        double[] x = { 0.19, 0.64, 1.19}; // x values
        double[] y = { 0.03, 0.91, 0.79 }; // y values

        SplineInterpolator interpolator = new SplineInterpolator();
        PolynomialSplineFunction spline = interpolator.interpolate(x, y);

        ArmFollowSplineTimed command = new ArmFollowSplineTimed(arm, spline, true, 1.5);

        return command;
    }


    public static Command placeConeL1(Arm arm){
        //(0.45, -0.1)

        ArmToPointLinearTimed command = new ArmToPointLinearTimed(arm,0.45,-0.1,1);

        return command;
    }

    public static Command retractConeL1(Arm arm){
        //(0.19, 0.3)
        ArmToPointLinearTimed command = new ArmToPointLinearTimed(arm,0.19, 0.3,1);

        return command;
    }


}
