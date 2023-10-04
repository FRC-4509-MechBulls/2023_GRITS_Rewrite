// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.Optional;

import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.OperatorConstants.*;


public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */

  SwerveModule frontLeft = new SwerveModule(frontLeftDriveID,frontLeftTurningID,false,true,0,frontLeftOffsetRad);
  SwerveModule frontRight = new SwerveModule(frontRightDriveID,frontRightTurningID,false,true,1,frontRightOffsetRad);

  SwerveModule rearLeft = new SwerveModule(rearLeftDriveID,rearLeftTurningID,false,true,2,rearLeftOffsetRad);
  SwerveModule rearRight = new SwerveModule(rearRightDriveID,rearRightTurningID,false,true,3,rearRightOffsetRad);

  WPI_Pigeon2 pigeon = new WPI_Pigeon2(40);
  AHRS navx = new AHRS(SPI.Port.kMXP);

  SwerveDrivePoseEstimator odometry;
VisionSubsystem visionSubsystem;

  Field2d field2d = new Field2d();

  Field2d visionField = new Field2d();



  public SwerveSubsystem(VisionSubsystem visionSubsystem) {
    pigeon.configFactoryDefault();
    pigeon.zeroGyroBiasNow();
    odometry = new SwerveDrivePoseEstimator(kinematics,pigeon.getRotation2d(),getPositions(),new Pose2d());
    odometry.setVisionMeasurementStdDevs(VecBuilder.fill(5, 5, Units.degreesToRadians(15)));
    this.visionSubsystem = visionSubsystem;
  }

  public SwerveModulePosition[] getPositions(){
    return new SwerveModulePosition[]{frontLeft.getPosition(),frontRight.getPosition(),rearLeft.getPosition(),rearRight.getPosition()};
  }
public void joystickDrive(double joystickX, double joystickY, double rad){
  if(Math.abs(joystickX)<0.05) joystickX = 0;
  if(Math.abs(joystickY)<0.05) joystickY = 0;
  if(Math.abs(rad)<0.05){
    rad = 0;
  }



  double hypot = Math.hypot(joystickX,joystickY);
  double dir = Math.atan2(joystickY,joystickX);

  hypot = Math.pow(hypot,driveExponent) * driveMaxSpeed;

  joystickX = hypot*Math.cos(dir);
  joystickY = hypot*Math.sin(dir);

  if(rad>0){
    rad = Math.pow(rad,turnExponent) * turnMaxSpeed;
  }else{
    rad = -Math.pow(-rad,turnExponent) * turnMaxSpeed;
  }


/* Angular adjustment stuff */
  if(Math.abs(rad)>radPerSecondDeadband || lastStillHeading.getDegrees() == 0){
    lastStillHeading = Rotation2d.fromDegrees(pigeon.getAngle());
  }
 // double radFeed = MB_Math.angleDiffDeg(pigeon.getAngle(),lastStillHeading.getDegrees() ) * (1.0/50) ;  //this was responsible for the slap
  //radFeed = MB_Math.absClamp(radFeed,radFeedClamp);


  drive(-joystickY ,-joystickX ,-rad);
}

Rotation2d lastStillHeading = new Rotation2d();
public void drive(double xMeters,double yMeters, double rad){
  setStates(kinematics.toSwerveModuleStates(new ChassisSpeeds(xMeters,yMeters,rad)));
}

void setStates(SwerveModuleState[] states){
  frontLeft.setState(states[0]);
  frontRight.setState(states[1]);
  rearLeft.setState(states[2]);
  rearRight.setState(states[3]);
}

void updatePoseFromVision(){
    Optional<EstimatedRobotPose> result = visionSubsystem.getEstimatedGlobalPose(odometry.getEstimatedPosition());
    if(result.isPresent()){
      odometry.addVisionMeasurement(result.get().estimatedPose.toPose2d(), result.get().timestampSeconds);
      SmartDashboard.putNumber("lastVisionX",result.get().estimatedPose.getX());
      SmartDashboard.putNumber("lastVisionY",result.get().estimatedPose.getY());
      visionField.setRobotPose(result.get().estimatedPose.toPose2d());
      SmartDashboard.putData("visionField",visionField);

      SmartDashboard.putNumberArray("visionPose3D",new double[]{
              result.get().estimatedPose.getX(),
              result.get().estimatedPose.getY(),
              result.get().estimatedPose.getZ(),
              result.get().estimatedPose.getRotation().getQuaternion().getW(),
              result.get().estimatedPose.getRotation().getQuaternion().getX(),
              result.get().estimatedPose.getRotation().getQuaternion().getY(),
              result.get().estimatedPose.getRotation().getQuaternion().getZ(),
      });

    }
    //add vision measurement if present while passing in current reference pose
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

odometry.updateWithTime(Timer.getFPGATimestamp(),pigeon.getRotation2d(),getPositions());

updatePoseFromVision();


  field2d.setRobotPose(odometry.getEstimatedPosition());
  //SmartDashboard.putData("field",field2d);

    SmartDashboard.putNumber("odom_x",odometry.getEstimatedPosition().getX());
    SmartDashboard.putNumber("odom_y",odometry.getEstimatedPosition().getY());
    SmartDashboard.putNumber("odom_deg",odometry.getEstimatedPosition().getRotation().getDegrees());


  }

  public void resetOdometry(){
    odometry.resetPosition(pigeon.getRotation2d(),getPositions(),new Pose2d());
  }

  public void driveToPose(Pose2d desiredPose){
    Pose2d myPose = odometry.getEstimatedPosition();

    double xDiff = desiredPose.getX() - myPose.getX();
    double yDiff = desiredPose.getY() - myPose.getY();

    double angDiff = desiredPose.getRotation().getRadians() - myPose.getRotation().getRadians(); //difference in angles


    double hypot = Math.hypot(xDiff,yDiff);
    double angleOfDiff = Math.atan2(yDiff,xDiff); //angle between two poses

    double angleOfTravel = angleOfDiff + angDiff;



    hypot*= translationkP;
    angDiff*=rotationkP;

    if(hypot > maxTranslation)
      hypot = maxTranslation;

    if(angDiff>maxRotation)
      angDiff = maxRotation;




    drive( hypot*Math.cos(angleOfTravel), hypot * Math.sin(angleOfTravel), angDiff);


  }
}
