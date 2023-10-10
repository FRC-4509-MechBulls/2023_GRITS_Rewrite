// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ArmConstants.*;
import frc.robot.Constants;
import frc.robot.Robot;

import java.awt.geom.Point2D;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
 // StageOneSub stageOneSub = new StageOneSub();

  StageOneSub stageOneSub = new StageOneSub();
  StageTwoSub stageTwoSub = new StageTwoSub();

  double stageOneReference = 0;
  double stageTwoReference = 0;

  NetworkTable armSimTable = NetworkTableInstance.getDefault().getTable("arm-sim");

  public Arm() {
      SmartDashboard.putNumber("stageOneRef",90);
  //    SmartDashboard.putNumber("stageTwoRef",15);

  }

   public Rotation2d getStageOneAngle(){
           if(Robot.isSimulation())
           return Rotation2d.fromRadians(stageOneReference);

        return stageOneSub.getAngle();
    }
    public Rotation2d getStageTwoAngle(){
           if(Robot.isSimulation())
               return Rotation2d.fromRadians(stageTwoReference);

        return stageTwoSub.getAngle();
    }

    public void setStageOneAngle(Rotation2d angle){
        stageOneSub.setAngle(angle);
        stageOneReference = angle.getRadians();
    }

    public void setStageTwoAngle(Rotation2d angle){
        stageTwoSub.setAngle(angle);
        stageTwoReference = angle.getRadians();
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

   double[] efPosition = calculateEFPosition(getStageOneAngle().getRadians(), getStageTwoAngle().getRadians());

   SmartDashboard.putNumber("stageOneAngle",getStageOneAngle().getDegrees());
   SmartDashboard.putNumber("stageTwoAngle",getStageTwoAngle().getDegrees());


    SmartDashboard.putNumber("ef_x",efPosition[0]);
    SmartDashboard.putNumber("ef_y",efPosition[1]);


    Rotation2d[] calculatedArmAngles = calculateArmAngles(efPosition[0],efPosition[1]);

    SmartDashboard.putNumber("stageOneRad_calculated",calculatedArmAngles[0].getDegrees());
    SmartDashboard.putNumber("stageTwoRad_calculated",calculatedArmAngles[1].getDegrees());


      if(Robot.isSimulation()){
          armSimTable.putValue("stageOneReference", NetworkTableValue.makeDouble(stageOneReference));
          armSimTable.putValue("stageTwoReference", NetworkTableValue.makeDouble(stageTwoReference));
          SmartDashboard.putNumber("stageTwoRef",stageTwoReference);
      }
      else{
          armSimTable.putValue("stageOneReference", NetworkTableValue.makeDouble(stageOneSub.getAngle().getRadians())); //not reference but real (get real bro)
          armSimTable.putValue("stageTwoReference", NetworkTableValue.makeDouble(stageTwoSub.getAngle().getRadians()));
      }



 //   stageOneSub.setAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("stageOneRef",90)));
 //   stageTwoSub.setAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("stageTwoRef",15)));

      stageTwoSub.setAbsoluteAngleDeg(stageOneSub.getAngle().getDegrees() + stageTwoSub.getAngle().getDegrees() - 180); //we do a little bit of trolling

    //  stageOneSub.setAngle(Rotation2d.fromDegrees(90));
    //  stageTwoSub.setAngle(Rotation2d.fromDegrees(15));




  }
}
