package frc.robot.commands.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.MBUtils;
import frc.robot.subsystems.arm.Arm;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;


public class ArmToPointLinearTimed extends CommandBase {
    private final Arm arm;

    double stageOneInitialRad;
    double stageTwoInitialRad;

    double desiredX;
    double desiredY;

    double secondsToTake;

    double initTime;
    public ArmToPointLinearTimed(Arm arm, double desiredX, double desiredY, double secondsToTake) {
        this.arm = arm;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.arm);


        this.secondsToTake = secondsToTake;
        this.desiredX = desiredX;
        this.desiredY = desiredY;

    }

    @Override
    public void initialize() {

        initTime = Timer.getFPGATimestamp();

        stageOneInitialRad = arm.getStageOneAngle().getRadians();
        stageTwoInitialRad = arm.getStageTwoAngle().getRadians();


    }

    @Override
    public void execute() {

        if((Timer.getFPGATimestamp()-initTime)>secondsToTake) return;

        double[] initialPosition = Arm.calculateEFPosition(stageOneInitialRad,stageTwoInitialRad);

        double interpolatedX = MBUtils.lerp(initialPosition[0],desiredX,(Timer.getFPGATimestamp()-initTime)/secondsToTake);
        double interpolatedY = MBUtils.lerp(initialPosition[1],desiredY,(Timer.getFPGATimestamp()-initTime)/secondsToTake);

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
