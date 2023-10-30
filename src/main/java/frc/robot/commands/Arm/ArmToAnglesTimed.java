package frc.robot.commands.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.MBUtils;
import frc.robot.subsystems.arm.Arm;


public class ArmToAnglesTimed extends CommandBase {
    private final Arm arm;

    double stageOneInitialRad;
    double stageTwoInitialRad;


    double stageTwoFinalRad;
    double stageOneFinalRad;

    double secondsToTake;

    double initTime;
    public ArmToAnglesTimed(Arm arm, Rotation2d stageOne, Rotation2d stageTwo, double secondsToTake) {
        this.arm = arm;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.arm);

        this.stageOneFinalRad = stageOne.getRadians();
        this.stageTwoFinalRad = stageTwo.getRadians();

        this.secondsToTake = secondsToTake;

    }

    @Override
    public void initialize() {

        initTime = Timer.getFPGATimestamp();

        stageOneInitialRad = arm.getStageOneAngle().getRadians();
        stageTwoInitialRad = arm.getStageTwoAngle().getRadians();


    }

    @Override
    public void execute() {

        double interpolatedStageOneRad = MBUtils.lerp(stageOneInitialRad, stageOneFinalRad, (Timer.getFPGATimestamp()-initTime)/secondsToTake);
        double interpolatedStageTwoRad = MBUtils.lerp(stageTwoInitialRad, stageTwoFinalRad, (Timer.getFPGATimestamp()-initTime)/secondsToTake);

        arm.setStageOneAngle(Rotation2d.fromRadians(interpolatedStageOneRad));
        arm.setStageTwoAngle(Rotation2d.fromRadians(interpolatedStageTwoRad));
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
