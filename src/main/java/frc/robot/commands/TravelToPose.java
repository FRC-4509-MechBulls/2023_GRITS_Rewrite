package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.MBUtils;
import frc.robot.subsystems.drive.SwerveSubsystem;


public class TravelToPose extends CommandBase {
    private final SwerveSubsystem swerveSubsystem;

    Pose2d initialPose;
    Pose2d desiredPose;

    double initTime;
    double secondsToTake;
    double overtime;

    public TravelToPose(SwerveSubsystem swerveSubsystem, Pose2d desiredPose, double secondsToTake, double overtime) {
        this.swerveSubsystem = swerveSubsystem;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.swerveSubsystem);

        this.desiredPose = desiredPose;
        this.secondsToTake = secondsToTake;
        this.overtime = overtime;
    }

    public TravelToPose (SwerveSubsystem swerveSubsystem, Pose2d desiredPose, double secondsToTake) {
        this(swerveSubsystem,desiredPose,secondsToTake,0);
    }

    @Override
    public void initialize() {
        initialPose = swerveSubsystem.getOdometry().getEstimatedPosition();
        initTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if(Timer.getFPGATimestamp() - initTime > secondsToTake){
            swerveSubsystem.driveToPose(desiredPose);
            return;
        }

        double interpolatedX = MBUtils.lerp(initialPose.getX(),desiredPose.getX(),(Timer.getFPGATimestamp() - initTime) / secondsToTake);
        double interpolatedY = MBUtils.lerp(initialPose.getY(),desiredPose.getY(),(Timer.getFPGATimestamp() - initTime) / secondsToTake);
        double interpolatedAngle = MBUtils.slerp(initialPose.getRotation().getRadians(), desiredPose.getRotation().getRadians(), (Timer.getFPGATimestamp() - initTime) / secondsToTake);

        swerveSubsystem.driveToPose(new Pose2d(interpolatedX,interpolatedY, Rotation2d.fromRadians(interpolatedAngle)));

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return Timer.getFPGATimestamp() - initTime  > secondsToTake + overtime;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
