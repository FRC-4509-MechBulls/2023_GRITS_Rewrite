// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.time.Clock;
import java.util.ArrayList;

import static frc.robot.Constants.ArmConstants.*;

public class StageTwoSub extends SubsystemBase {
  /** Creates a new StageTwoSub. */


     CANSparkMax primary;
     CANSparkMax secondary;
     SparkMaxPIDController pidController;

    SparkMaxAbsoluteEncoder encoder;

  public StageTwoSub() {
    primary = new CANSparkMax(stageTwoPrimaryId, CANSparkMaxLowLevel.MotorType.kBrushless);
    secondary = new CANSparkMax(stageTwoSecondaryId, CANSparkMaxLowLevel.MotorType.kBrushless);

   // ArrayList<REVLibError> primaryErrors = new ArrayList<>();

    primary.restoreFactoryDefaults();
    secondary.restoreFactoryDefaults();

    primary.setCANTimeout(1000);
    secondary.setCANTimeout(1000);

    primary.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10);
    secondary.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10);

    primary.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 100);
    secondary.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 100);

    primary.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 100);
    secondary.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 100);

    primary.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 100);
    secondary.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 100);

    primary.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus4, 100);
    secondary.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus4, 100);

    primary.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus5, 100);
    secondary.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus5, 100);

    primary.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus6, 100);
    secondary.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus6, 100);

    encoder = primary.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    pidController  = primary.getPIDController();

    primary.enableVoltageCompensation(12.0);
    secondary.enableVoltageCompensation(12.0);

    primary.setSecondaryCurrentLimit(stageTwoSecondaryCurrentLimit);
    secondary.setSecondaryCurrentLimit(stageTwoSecondaryCurrentLimit);
    primary.setSmartCurrentLimit(stageTwoSmartCurrentLimit);
    secondary.setSmartCurrentLimit(stageTwoSmartCurrentLimit);



    secondary.follow(primary, true);

    encoder.setPositionConversionFactor(2*Math.PI);
    encoder.setVelocityConversionFactor(2*Math.PI);
  //  encoder.setZeroOffset(Units.radiansToDegrees(267.3+90)));
    encoder.setInverted(true);

    primary.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,(float)Units.degreesToRadians(10));
    primary.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward,(float)Units.degreesToRadians(180));
    primary.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    primary.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);



    pidController.setFeedbackDevice(encoder);
    pidController.setP(stageTwokP,0);
    pidController.setI(stageTwokI,0);
    pidController.setD(stageTwokD,0);
    pidController.setOutputRange(-0.6,0.6);
    pidController.setPositionPIDWrappingEnabled(false);

    pidController.setIZone(0.3,0);
    pidController.setIMaxAccum(0.05,0);



    primary.burnFlash();
    secondary.burnFlash();




  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  //  pidController.setReference(Units.degreesToRadians(35), CANSparkMax.ControlType.kPosition);

   //   SmartDashboard.putNumber("stageTwoEncoder",getAngle().getDegrees());
    double error = referenceRad - getAngle().getRadians();
    SmartDashboard.putNumber("stageTwoError", error);
    SmartDashboard.putNumber("stageTwoIAccum",pidController.getIAccum());

   // if(error>0.25){
   //   pidController.setIAccum(0);
   // }

  }

  double referenceRad = 0;
  public void setAngle(Rotation2d angle){
    referenceRad = angle.getRadians();

    pidController.setReference(angle.getRadians(), CANSparkMax.ControlType.kPosition,0,0);

  }


  public Rotation2d getAngle(){
    return Rotation2d.fromRadians(encoder.getPosition());
  }
}
