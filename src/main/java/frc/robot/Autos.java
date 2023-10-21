package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.TravelToPose;
import frc.robot.subsystems.ArmState;
import frc.robot.subsystems.StateControllerSub;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.SwerveSubsystem;
import static frc.robot.Constants.FieldConstants.*;

public class Autos {

    public static Command skibidiAutonomous(SwerveSubsystem swerveSubsystem, Arm arm, StateControllerSub stateControllerSub, boolean flippedForRed){

        double initX = blueAlignmentX;
        double initY = nodeYValues[0];


        int beBackwardsNow = 1;
        double angleToAddIfBackwards = 0;

        if(flippedForRed){
            beBackwardsNow = -1;
            initX = redAlignmentX;
            angleToAddIfBackwards = 180;
        }

        Rotation2d initHeading = Rotation2d.fromDegrees(180 + angleToAddIfBackwards);

        final double initXFinal = initX;



        //1.709, 0.239
        //3.970, 0.159
        //5.342, 0.400

        Command resetPose = new InstantCommand(()->swerveSubsystem.resetOdometry(new Pose2d(initXFinal, initY, initHeading)));
        Command setInitialArmState = new InstantCommand(()->stateControllerSub.setOverallStateSafe(new ArmState(StateControllerSub.AgArmMode.HOLDING, StateControllerSub.ItemType.CONE, StateControllerSub.ItemIsFallen.FALLEN_CONE, StateControllerSub.PlacementLevel.LEVEL3)));
        Command goToPose1 = new TravelToPose(swerveSubsystem, new Pose2d(initX + 1.709 * beBackwardsNow, initY+ 0.239, Rotation2d.fromDegrees(0*beBackwardsNow + angleToAddIfBackwards)), 1,0);
        Command goToPose2 = new TravelToPose(swerveSubsystem, new Pose2d(initX + 3.970 * beBackwardsNow, initY+ 0.159, Rotation2d.fromDegrees(0*beBackwardsNow + angleToAddIfBackwards)), 1,0);
        Command setIntaking = new InstantCommand(stateControllerSub::setArmModeToIntaking);
        Command setHeading = new TravelToPose(swerveSubsystem, new Pose2d(initX + 3.970 * beBackwardsNow, initY+ 0.159, Rotation2d.fromDegrees(10*beBackwardsNow + angleToAddIfBackwards)), 0.2,0);
        Command goToConePickup = new TravelToPose(swerveSubsystem, new Pose2d(initX + 5.342 * beBackwardsNow, initY+ 0.400, Rotation2d.fromDegrees(10*beBackwardsNow + angleToAddIfBackwards)), 2,0);
        Command setHolding = new InstantCommand(stateControllerSub::setArmModeToHolding);

        Command returnToPose2 = new TravelToPose(swerveSubsystem, new Pose2d(initX + 3.970 * beBackwardsNow, initY+ 0.159, Rotation2d.fromDegrees(180*beBackwardsNow + angleToAddIfBackwards)), 1,0);
        Command returnToPose1 = new TravelToPose(swerveSubsystem, new Pose2d(initX + Units.inchesToMeters(12)*beBackwardsNow, initY+ 0.239, Rotation2d.fromDegrees(180*beBackwardsNow + angleToAddIfBackwards)), 2,0);

        Command crabWalk = new TravelToPose(swerveSubsystem, new Pose2d(initX + Units.inchesToMeters(12) * beBackwardsNow, nodeYValues[2], Rotation2d.fromDegrees(180*beBackwardsNow + angleToAddIfBackwards)), 2,0.25);

        Command placeCone = new InstantCommand(stateControllerSub::setArmModeToPlacing);

        Command goToWhereWeWant = new TravelToPose(swerveSubsystem, new Pose2d(initX, nodeYValues[2], Rotation2d.fromDegrees(180*beBackwardsNow + angleToAddIfBackwards)), 1,2);

        Command ejectCone = new InstantCommand(stateControllerSub::setArmModeToPostPlacing);



        return resetPose.andThen(setInitialArmState).andThen(goToPose1).andThen(goToPose2).andThen(setIntaking).andThen(setHeading).andThen(goToConePickup).andThen(setHolding).andThen(returnToPose2).andThen(returnToPose1).andThen(crabWalk).andThen(placeCone).andThen(goToWhereWeWant).andThen(ejectCone);
    }



}
