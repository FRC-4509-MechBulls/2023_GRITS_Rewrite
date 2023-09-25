// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ArmConstants.*;

public class StageOneSub extends SubsystemBase {
  /** Creates a new StageOneSub. */

  TalonSRX primary;
  TalonSRX secondary;


  public StageOneSub() {
    primary = new TalonSRX(stageOneRightId);
    secondary = new TalonSRX(stageOneLeftId);

    primary.configFactoryDefault(1000);
    secondary.configFactoryDefault(1000);

    primary.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,0,1000);
    primary.configFeedbackNotContinuous(true,1000);

    primary.setNeutralMode(NeutralMode.Coast);
    secondary.setNeutralMode(NeutralMode.Coast);


    primary.configVoltageCompSaturation(12.0,1000);
    primary.enableVoltageCompensation(true);
    secondary.configVoltageCompSaturation(12.0,1000);
    secondary.enableVoltageCompensation(true);

    primary.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,continuousCurrentLimit,peakCurrentLimit,peakCurrentTime),1000);
    secondary.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,continuousCurrentLimit,peakCurrentLimit,peakCurrentTime),1000);


    primary.setSensorPhase(false);
    primary.setInverted(true);

    secondary.follow(primary);
    secondary.setInverted(InvertType.OpposeMaster);

    primary.configForwardSoftLimitThreshold((int)radToNativeSensorPosition(Units.degreesToRadians(80)),1000);
    primary.configReverseSoftLimitThreshold((int)radToNativeSensorPosition(Units.degreesToRadians(45)),1000);
    primary.configForwardSoftLimitEnable(true,1000);
    primary.configReverseSoftLimitEnable(true);



  //  primary.config_kP(0,stageOnekP,1000);
  //  primary.config_kI(0,stageOnekI,1000);
  //  primary.config_kD(0,stageOnekD,1000);
  //  primary.config_kF(0,stageOnekF,1000);





  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("stageOneAngle",getAngle().getDegrees());
    SmartDashboard.putNumber("rawSensor",primary.getSelectedSensorPosition());
  }

  public void setPercentOutput(double output){
    if(output>0.5)output = 0.5;
    if(output<-0.5)output = -0.5;
    primary.set(ControlMode.PercentOutput,output);
  }

  public Rotation2d getAngle(){ //gets angle from horizontal

    double angleRad = nativeSensorPositionToRad(primary.getSelectedSensorPosition());

    return Rotation2d.fromRadians(angleRad);
  }

  public double nativeSensorPositionToRad(double sensorReading){
    sensorReading *= stageOneEncoderTicksToRadians;

    sensorReading = (2*Math.PI) + sensorReading;

    if(sensorReading>Math.PI)
      sensorReading = sensorReading - 2*Math.PI;

    sensorReading+= Units.degreesToRadians(22.5);
    return sensorReading;
  }

  public double radToNativeSensorPosition(double angleRad) {

    angleRad -= Units.degreesToRadians(22.5);


    if (angleRad < -Math.PI)
      angleRad += 2 * Math.PI;

    angleRad -= 2 * Math.PI;

    angleRad /= stageOneEncoderTicksToRadians;

    return angleRad;
  }


}
