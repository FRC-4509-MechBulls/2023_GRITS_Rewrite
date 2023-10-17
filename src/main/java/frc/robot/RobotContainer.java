// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.StateControllerSub;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.StageOneSub;
import frc.robot.subsystems.arm.StageTwoSub;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.drive.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  CommandXboxController driver = new CommandXboxController(0);

  VisionSubsystem visionSub = new VisionSubsystem();
  SwerveSubsystem swerveSubsystem = new SwerveSubsystem(visionSub);
  RunCommand drive = new RunCommand(()->swerveSubsystem.joystickDrive(driver.getLeftX(),driver.getLeftY(),driver.getRightX()),swerveSubsystem);




  Arm arm = new Arm();

  StateControllerSub stateController = new StateControllerSub(arm);

  Command stageOneToHolding = new InstantCommand(()->arm.setStageOneAngle(Rotation2d.fromDegrees(90)));
  Command stageTwoToHolding = new InstantCommand(()->arm.setStageTwoAngle(Rotation2d.fromDegrees(15)));
  Command setArmToHolding = stageOneToHolding.andThen(stageTwoToHolding);






  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

//    driver.rightTrigger(0.5).onTrue(ArmCommands.intakeConeFallen(arm));
//    driver.leftTrigger(0.5).onTrue(ArmCommands.retractFromConeFallen(arm));

    swerveSubsystem.setDefaultCommand(drive);


    driver.start().onTrue(new InstantCommand(swerveSubsystem::resetOdometry));


    driver.a().onTrue(new InstantCommand(stateController::setArmModeToIntaking));
    driver.b().onTrue(new InstantCommand(stateController::setArmModeToHolding));
    driver.y().onTrue(new InstantCommand(stateController::setArmModeToPlacing));

    driver.povUp().onTrue(new InstantCommand(stateController::setArmLevelBottom));
    driver.povRight().onTrue(new InstantCommand(stateController::setArmLevelMiddle));
    driver.povLeft().onTrue(new InstantCommand(stateController::setArmLevelMiddle));
    driver.povDown().onTrue(new InstantCommand(stateController::setArmLevelTop));
    
    driver.leftTrigger(0.5).onTrue(new InstantCommand(stateController::setItemConeFallen));

    driver.leftBumper().onTrue(new InstantCommand(stateController::setItemConeUpright));
    driver.rightBumper().onTrue(new InstantCommand(stateController::setItemCubeFallen));


    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
   // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
      return null;
  }
}
