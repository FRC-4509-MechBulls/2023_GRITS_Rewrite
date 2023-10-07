// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Arm.ArmToAnglesTimed;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.SwerveSubsystem;

public final class Autos {
  /** Example static factory for an autonomous command. */

  public static Command funkyFreshAuto(SwerveSubsystem swerveSubsystem, Arm arm) {

    Command reset = new InstantCommand(swerveSubsystem::resetOdometry);
    Command stageOneToHolding = new InstantCommand(()->arm.setStageOneAngle(Rotation2d.fromDegrees(90)));
    Command stageTwoToHolding = new InstantCommand(()->arm.setStageTwoAngle(Rotation2d.fromDegrees(15)));

    Command setArmToHolding = stageOneToHolding.andThen(stageTwoToHolding);


    Command pos2 = new TravelToPose(swerveSubsystem,new Pose2d(1.8,0,Rotation2d.fromDegrees(0)),1);
    Command pos3 = new TravelToPose(swerveSubsystem, new Pose2d(1.8, -1, Rotation2d.fromDegrees(-90)),1);

    Command stopPlease = new InstantCommand(()->swerveSubsystem.drive(0,0,0));

    Command lerpTHEArm = new ArmToAnglesTimed(arm, Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(97),2);


   return reset.andThen(setArmToHolding).andThen(pos2).andThen(pos3.andThen(stopPlease)).andThen(lerpTHEArm);
 //   return null;
  }

}
