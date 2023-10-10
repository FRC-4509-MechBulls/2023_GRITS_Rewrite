package frc.robot.commands.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.MBUtils;
import frc.robot.subsystems.arm.Arm;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;


public class ArmFollowSplineTimed extends CommandBase {
    private final Arm arm;

    double stageOneInitialRad;
    double stageTwoInitialRad;

PolynomialSplineFunction spline;

    double secondsToTake;

    double initTime;
    boolean followBackwards;
    public ArmFollowSplineTimed(Arm arm, PolynomialSplineFunction spline, boolean followBackwards, double secondsToTake) {
        this.arm = arm;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.arm);


        this.secondsToTake = secondsToTake;
        this.spline = spline;
        this.followBackwards = followBackwards;

    }

    @Override
    public void initialize() {

        initTime = Timer.getFPGATimestamp();

        stageOneInitialRad = arm.getStageOneAngle().getRadians();
        stageTwoInitialRad = arm.getStageTwoAngle().getRadians();


    }

    @Override
    public void execute() {

     //   double interpolatedStageOneRad = MBUtils.lerp(stageOneInitialRad, stageOneFinalRad, (Timer.getFPGATimestamp()-initTime)/secondsToTake);
     //   double interpolatedStageTwoRad = MBUtils.lerp(stageTwoInitialRad, stageTwoFinalRad, (Timer.getFPGATimestamp()-initTime)/secondsToTake);

        double[] knots = spline.getKnots();

        double min = knots[0];
        double max = knots[knots.length-1];

        double t = (Timer.getFPGATimestamp()-initTime)/secondsToTake;

        if(followBackwards)
            t = 1-t;

        if(t>1) t = 1; //in case command doesn't get killed in time
        if(t<0) t = 0;

        double interpolatedX = MBUtils.lerp(min,max,t);
        double interpolatedY = spline.value(t*(max-min)+min);

        Rotation2d[] angles = Arm.calculateArmAngles(interpolatedX,interpolatedY);


        arm.setStageOneAngle(angles[0]);
        arm.setStageTwoAngle(angles[1]);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return Timer.getFPGATimestamp() - initTime  > secondsToTake;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
