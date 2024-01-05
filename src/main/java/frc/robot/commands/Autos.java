package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.SwerveSubsystem;
import static frc.robot.Constants.FieldConstants.*;

public class Autos {

    public static Command ballerAuto(SwerveSubsystem swerveSubsystem){
        Command resetPose = new InstantCommand(swerveSubsystem::resetOdometry);

        TravelToPose pose1 = new TravelToPose(swerveSubsystem, new Pose2d(1,0,Rotation2d.fromDegrees(0)),1,0);
        TravelToPose pose2 = new TravelToPose(swerveSubsystem, new Pose2d(1,1,Rotation2d.fromDegrees(0)),1,0);
        TravelToPose pose3 = new TravelToPose(swerveSubsystem, new Pose2d(0,1,Rotation2d.fromDegrees(0)),1,0);
        TravelToPose pose4 = new TravelToPose(swerveSubsystem, new Pose2d(0,0,Rotation2d.fromDegrees(0)),1,0);

        Command waitCommand = new WaitCommand(1);

        TravelToPose pose5 = new TravelToPose(swerveSubsystem, new Pose2d(0,4,Rotation2d.fromDegrees(90)),4,0);

         return resetPose.andThen(pose1).andThen(pose2).andThen(pose3.andThen(pose4)).andThen(waitCommand).andThen(pose5);
       // return resetPose.andThen(pose1);
    }

}
