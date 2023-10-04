// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ArmConstants.*;
import frc.robot.Constants;

import java.awt.geom.Point2D;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
 // StageOneSub stageOneSub = new StageOneSub();

  StageOneSub stageOneSub = new StageOneSub();
  StageTwoSub stageTwoSub = new StageTwoSub();


  public Arm() {

//      SmartDashboard.putNumber("stageOneRef",90);
//      SmartDashboard.putNumber("stageTwoRef",15);
  }



  public static double[] calculateEFPosition(double ang1, double ang2) {
    double stageOneEndX = stageOneLength * Math.cos(ang1);
    double stageOneEndY = stageOneLength * Math.sin(ang1);

    double stageTwoEndX = stageOneEndX + stageTwoLength * -Math.cos(ang1 + ang2);
    double stageTwoEndY = stageOneEndY + stageTwoLength * -Math.sin(ang1 + ang2);

    return new double[]{stageTwoEndX, stageTwoEndY};
  }


  public static Rotation2d[] calculateArmAngles(double x, double y){
    double hypot = Math.hypot(x,y);
    double stageTwoAngle = Math.acos((Math.pow(stageTwoLength,2) + Math.pow(stageOneLength,2) - Math.pow(hypot,2)) / (2*stageOneLength * stageTwoLength) );
    double stageOneAngle = Math.acos((Math.pow(stageOneLength,2) + Math.pow(hypot,2) - Math.pow(stageTwoLength,2)) / (2*stageOneLength * hypot));

    stageOneAngle+=Math.atan2(y,x);

    return new Rotation2d[] {Rotation2d.fromRadians(stageOneAngle),Rotation2d.fromRadians(stageTwoAngle)};
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

   double[] efPosition = calculateEFPosition(stageOneSub.getAngle().getRadians(), stageTwoSub.getAngle().getRadians());

   SmartDashboard.putNumber("stageOneAngle",stageOneSub.getAngle().getDegrees());
   SmartDashboard.putNumber("stageTwoAngle",stageTwoSub.getAngle().getDegrees());


    SmartDashboard.putNumber("ef_x",efPosition[0]);
    SmartDashboard.putNumber("ef_y",efPosition[1]);


    Rotation2d[] calculatedArmAngles = calculateArmAngles(efPosition[0],efPosition[1]);

    SmartDashboard.putNumber("stageOneRad_calculated",calculatedArmAngles[0].getDegrees());
    SmartDashboard.putNumber("stageTwoRad_calculated",calculatedArmAngles[1].getDegrees());




 //   stageOneSub.setAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("stageOneRef",70)));
 //   stageTwoSub.setAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("stageTwoRef",15)));

      stageOneSub.setAngle(Rotation2d.fromDegrees(90));
      stageTwoSub.setAngle(Rotation2d.fromDegrees(15));


  }
}
