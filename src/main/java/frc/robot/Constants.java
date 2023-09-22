// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    public static final double driveExponent = 1.8;
    public static final double driveMaxSpeed = 0.5; //5
    public static final double turnExponent = 1.8;
    public static final double turnMaxSpeed = 1; //11


    public static final double radFeedClamp = 0.5; //max heading adjustment speed
  }

  public static final class DriveConstants{

    /*Physical Characteristics*/
    public static final double TRACK_WIDTH = Units.inchesToMeters(23.625); //need to find
    // Distance between right and left wheels
    public static final double WHEEL_BASE = Units.inchesToMeters(23.625); //need to find


    // Distance between front and back wheels
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
    );


    public static final double wheelDiameterMeters = Units.inchesToMeters(3.8);
    public static final double wheelCircumferenceMeters = wheelDiameterMeters * Math.PI;
    public static final double driveMotorGearRatio = 6.75;
    public static final double turningGearRatio = 21.4286;
    public static final double falconTicks = 2048;

    public static final double radToFalcon = falconTicks / (2*Math.PI);

    public static final double radPerSecondDeadband = 0.01;




    /*Motor IDs and offsets */
    public static final int frontLeftDriveID = 1;
    public static final int frontLeftTurningID = 4;
    public static final double frontLeftOffsetRad = 0.945;

    public static final int frontRightDriveID = 6;
    public static final int frontRightTurningID = 5;
    public static final double frontRightOffsetRad = -5.1929;

    public static final int rearRightDriveID = 3;
    public static final int rearRightTurningID = 7;
    public static final double rearRightOffsetRad = 2.361;

    public static final int rearLeftDriveID = 8;
    public static final int rearLeftTurningID = 2;
    public static final double rearLeftOffsetRad = -2.296;





    /*Drive Motor Constants */
    public static final double driveMotorkP = 0.1;
    public static final double driveMotorkI = 0.0;
    public static final double driveMotorkD = 0.0;
    public static final double driveMotorkF = 0.045;

    public static final double driveNeutralDeadband = 0.01;



    /*Turning Motor Constants */

    public static final double turningMotorkP = 0.2;
    public static final double turningMotorkI = 0.0;
    public static final double turningMotorkD = 0.0;
    public static final double turningMotorkF = 0.0;





  }

}