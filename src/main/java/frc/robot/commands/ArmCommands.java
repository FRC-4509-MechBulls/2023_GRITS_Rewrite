package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Arm.ArmFollowSplineTimed;
import frc.robot.commands.Arm.ArmToPointLinearTimed;
import frc.robot.subsystems.arm.Arm;
import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

public class ArmCommands {

    public static Command placeConeL3(Arm arm){
        double[] x = { 0.19, 0.64, 1.19}; // x values
        double[] y = { 0.03, 0.91, 0.79 + Units.inchesToMeters(2) }; // y values
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

    public static Command placeConeL2(Arm arm){
        double[] x = { 0.19, 0.69, .86}; // x values
        double[] y = { 0.03, 0.48, 0.48 + Units.inchesToMeters(1.5)}; // y values
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
        double[] y = { 0.03, 0.91, 0.79 + Units.inchesToMeters(2) }; // y values

        SplineInterpolator interpolator = new SplineInterpolator();
        PolynomialSplineFunction spline = interpolator.interpolate(x, y);

        ArmFollowSplineTimed command = new ArmFollowSplineTimed(arm, spline, true, 1.5);

        return command;
    }


    public static Command placeConeL1(Arm arm){
        //(0.45, -0.1)

        ArmToPointLinearTimed command = new ArmToPointLinearTimed(arm,0.45,-0.1,0.25);

        return command;
    }

    public static Command quickHolding(Arm arm){
        //(0.19, 0.3)
        ArmToPointLinearTimed command = new ArmToPointLinearTimed(arm,0.19, 0.03,0.35);

        return command;
    }

    public static Command intakeConeUpright(Arm arm){
        //(0.45, -0.1)

        ArmToPointLinearTimed command = new ArmToPointLinearTimed(arm,0.505,-0.140,0.7); //0.505,-0.14

        return command;
    }

    public static Command intakeConeFallen(Arm arm){


        ArmToPointLinearTimed step2 = new ArmToPointLinearTimed(arm,0.804,-0.317 - Units.inchesToMeters(1.5),1); //0.505,-0.14

        return step2;
    }

    public static Command retractFromConeFallen(Arm arm){

        ArmToPointLinearTimed step2 = new ArmToPointLinearTimed(arm,0.19, 0.03,0.5); //0.505,-0.14

        return step2;
    }


    public static Command placeCubeL2orL3(Arm arm){
        double[] x = { 0.19, 0.604, 0.935 + Units.inchesToMeters(2)}; // x values
        double[] y = { 0.03, 0.421, 0.410 + Units.inchesToMeters(4)}; // y values

        SplineInterpolator interpolator = new SplineInterpolator();
        PolynomialSplineFunction spline = interpolator.interpolate(x, y);

        ArmFollowSplineTimed command = new ArmFollowSplineTimed(arm, spline, false, 0.75);

        return command;
    }

    public static Command placeCubeL1(Arm arm) {
        ArmToPointLinearTimed command = new ArmToPointLinearTimed(arm,0.185,-0.051,1);
        return command;
    }

    public static Command retractCubeFromL2orL3(Arm arm){
       // double[] x = { 0.19, 0.604, 0.935}; // x values
      //  double[] y = { 0.03, 0.421, 0.404 }; // y values
        double[] x = { 0.19, 0.604, 0.935 + Units.inchesToMeters(2)}; // x values
        double[] y = { 0.03, 0.421, 0.410 + Units.inchesToMeters(4)}; // y values

        SplineInterpolator interpolator = new SplineInterpolator();
        PolynomialSplineFunction spline = interpolator.interpolate(x, y);

        ArmFollowSplineTimed command = new ArmFollowSplineTimed(arm, spline, true, 0.75);

        return command;
    }

    public static Command intakeCube(Arm arm){
        //(0.645, -0.152)

        ArmToPointLinearTimed command = new ArmToPointLinearTimed(arm,0.645 - Units.inchesToMeters(6),-0.152,1);

        return command;
    }


}
