package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.TravelToPose;
import frc.robot.subsystems.ArmState;
import frc.robot.subsystems.StateControllerSub;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.SwerveSubsystem;
import static frc.robot.Constants.FieldConstants.*;

public class Autos {


    public static Command skibidiAutonomous(SwerveSubsystem swerveSubsystem, StateControllerSub stateControllerSub, boolean flippedForRed){

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

        Command placeConeHigh = new InstantCommand(stateControllerSub::setArmModeToPlacing).andThen(new WaitCommand(1.75)).andThen(new InstantCommand(stateControllerSub::setArmModeToPostPlacing)).andThen(new WaitCommand(0.20));
        Command goToHolding = new InstantCommand(stateControllerSub::setArmModeToHolding).andThen(new WaitCommand(0.5));

        Pose2d inBetweenPose = new Pose2d(
                (initX + (initX+1.709*beBackwardsNow)) / 2.0,
                (initY + (initY+ 0.239)) / 2.0,
                Rotation2d.fromDegrees(90*beBackwardsNow+angleToAddIfBackwards)
        );

        Command goHalfwayToPose1 = new TravelToPose(swerveSubsystem, inBetweenPose, 1, 0); //makes sure we don't kill any volunteers

        Command goToPose1 = new TravelToPose(swerveSubsystem, new Pose2d(initX + 1.709 * beBackwardsNow, initY+ 0.239, Rotation2d.fromDegrees(0*beBackwardsNow + angleToAddIfBackwards)), 0.75,0);
        Command setIntaking = new InstantCommand(stateControllerSub::setArmModeToIntaking);
        Command goToPose2 = new TravelToPose(swerveSubsystem, new Pose2d(initX + 3.970 * beBackwardsNow, initY+ 0.159, Rotation2d.fromDegrees(0*beBackwardsNow + angleToAddIfBackwards)), 2,0);

        Command setHeading = new TravelToPose(swerveSubsystem, new Pose2d(initX + 3.970 * beBackwardsNow, initY+ 0.159, Rotation2d.fromDegrees(10*beBackwardsNow + angleToAddIfBackwards)), 0,0);
        Command goToConePickup = new TravelToPose(swerveSubsystem, new Pose2d(initX + 5.342 * beBackwardsNow, initY+ 0.400, Rotation2d.fromDegrees(10*beBackwardsNow + angleToAddIfBackwards)), 1.2,0);
        Command setHolding = new InstantCommand(stateControllerSub::setArmModeToHolding);

        Command returnToPose2 = new TravelToPose(swerveSubsystem, new Pose2d(initX + 3.970 * beBackwardsNow, initY+ 0.159, Rotation2d.fromDegrees(180*beBackwardsNow + angleToAddIfBackwards)), 1.5,0);
        Command returnToPose1 = new TravelToPose(swerveSubsystem, new Pose2d(initX + Units.inchesToMeters(12)*beBackwardsNow, initY+ 0.239, Rotation2d.fromDegrees(180*beBackwardsNow + angleToAddIfBackwards)), 2,0);


        Command placeCone = new InstantCommand(stateControllerSub::setArmModeToPlacing);
        Command crabWalk = new TravelToPose(swerveSubsystem, new Pose2d(initX + Units.inchesToMeters(12) * beBackwardsNow, nodeYValues[2], Rotation2d.fromDegrees(180*beBackwardsNow + angleToAddIfBackwards)), 1,0);



        Command goToWhereWeWant = new TravelToPose(swerveSubsystem, new Pose2d(initX, nodeYValues[2], Rotation2d.fromDegrees(180*beBackwardsNow + angleToAddIfBackwards)), 2,0);

        Command ejectCone = new InstantCommand(stateControllerSub::setArmModeToPostPlacing).andThen(new WaitCommand(0.2).andThen(new InstantCommand(stateControllerSub::overrideEFStop)));




        return resetPose.andThen(setInitialArmState).andThen(placeConeHigh).andThen(goToHolding).andThen(goHalfwayToPose1).andThen(goToPose1).andThen(setIntaking).andThen(goToPose2).andThen(setHeading).andThen(goToConePickup).andThen(setHolding).andThen(returnToPose2).andThen(returnToPose1).andThen(placeCone).andThen(crabWalk).andThen(goToWhereWeWant).andThen(ejectCone);
    }


    public static Command skibidiWithoutSecondCone(SwerveSubsystem swerveSubsystem, StateControllerSub stateControllerSub, boolean flippedForRed){

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

        Command placeConeHigh = new InstantCommand(stateControllerSub::setArmModeToPlacing).andThen(new WaitCommand(1.75)).andThen(new InstantCommand(stateControllerSub::setArmModeToPostPlacing)).andThen(new WaitCommand(0.30));
        Command goToHolding = new InstantCommand(stateControllerSub::setArmModeToHolding).andThen(new WaitCommand(2));


        Command goToPose1 = new TravelToPose(swerveSubsystem, new Pose2d(initX + 1.709 * beBackwardsNow, initY+ 0.239, Rotation2d.fromDegrees(180*beBackwardsNow + angleToAddIfBackwards)), 1.75,0);
        Command goToPose2 = new TravelToPose(swerveSubsystem, new Pose2d(initX + 4.370 * beBackwardsNow, initY+ 0.159, Rotation2d.fromDegrees(180*beBackwardsNow + angleToAddIfBackwards)), 2,0);




        return resetPose.andThen(setInitialArmState).andThen(placeConeHigh).andThen(goToHolding).andThen(goToPose1).andThen(goToPose2);
    }

    public static Command placeLeaveBalanceAuto(SwerveSubsystem swerveSubsystem, StateControllerSub stateControllerSub, boolean flippedForRed){

        double initX = blueAlignmentX;
        double initY = nodeYValues[5];


        int beBackwardsNow = 1;
        double angleToAddIfBackwards = 0;



        if(flippedForRed){
            beBackwardsNow = -1;
            initX = redAlignmentX;
            angleToAddIfBackwards = 180;

        }

        Rotation2d initHeading = Rotation2d.fromDegrees(180 + angleToAddIfBackwards);

        final double initXFinal = initX;


        Command resetPose = new InstantCommand(()->swerveSubsystem.resetOdometry(new Pose2d(initXFinal, initY, initHeading)));
        Command setInitialArmState = new InstantCommand(()->stateControllerSub.setOverallStateSafe(new ArmState(StateControllerSub.AgArmMode.HOLDING, StateControllerSub.ItemType.CONE, StateControllerSub.ItemIsFallen.FALLEN_CONE, StateControllerSub.PlacementLevel.LEVEL3)));
        Command placeConeHigh = new InstantCommand(stateControllerSub::setArmModeToPlacing).andThen(new WaitCommand(1.75)).andThen(new InstantCommand(stateControllerSub::setArmModeToPostPlacing)).andThen(new WaitCommand(0.25));
        Command goToHolding = new InstantCommand(stateControllerSub::setArmModeToHolding).andThen(new WaitCommand(0.75));
        Command leaveCommunity = new TravelToPose(swerveSubsystem, new Pose2d(initX + 4.576 * beBackwardsNow, initY, initHeading),2,1);
        Command centerOnChargeStation = new TravelToPose(swerveSubsystem, new Pose2d(initX + 4.076 * beBackwardsNow, nodeYValues[4], initHeading),0.5,0.7); // 5.83
        Command goToChargeStation = new TravelToPose(swerveSubsystem,new Pose2d(initX + Units.inchesToMeters(81.6275) * beBackwardsNow, nodeYValues[4], initHeading), 0.3,1);
       // Command balance = new RunCommand(swerveSubsystem::autoBalanceForward,swerveSubsystem);
        Command balance = new Balance(swerveSubsystem, 7);


        return resetPose.andThen(setInitialArmState).andThen(placeConeHigh).andThen(goToHolding).andThen(leaveCommunity).andThen(centerOnChargeStation).andThen(goToChargeStation).andThen(balance);
    }



}
