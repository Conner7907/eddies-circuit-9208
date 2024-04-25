// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

 import edu.wpi.first.math.geometry.Translation2d;
 import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
 import edu.wpi.first.math.trajectory.TrapezoidProfile;
 import edu.wpi.first.math.util.Units;
 
public final class Constants {
                public static final class ModuleConstants {
                public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
                public static final double kDriveMotorGearRatio = 1 / 6.75;
                public static final double kTurningMotorGearRatio = 1 / 21.4286;
                public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
                public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
                public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60.0;
                public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60.0;
                public static final double kPTurning = .2;
}

  public static final class DriveConstants {

    public static final double kTrackWidth = Units.inchesToMeters(24.375);
    // Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(24.375);
    // Distance between front and back wheels
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),//(+,-)
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),//(+,+)
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),//(-,-)
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2));//(-,+)

    public static final int kFrontLeftDriveMotorPort = 9;
    public static final int kBackLeftDriveMotorPort = 6;
    public static final int kFrontRightDriveMotorPort = 12;
    public static final int kBackRightDriveMotorPort = 3;

    public static final int kFrontLeftTurningMotorPort = 8;
    public static final int kBackLeftTurningMotorPort = 5;
    public static final int kFrontRightTurningMotorPort = 11;
    public static final int kBackRightTurningMotorPort = 2;

    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kBackLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kBackRightTurningEncoderReversed = true;

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kBackRightDriveEncoderReversed = false;

    public static final int kFrontLeftDriveAbsoluteEncoderPort = 7;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 4;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 10;
    public static final int kBackRightDriveAbsoluteEncoderPort = 20;

    public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

    public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad =  1.014;// Units.degreesToRadians(58.1);   //.9695;
    public static final double kBackLeftDriveAbsoluteEncoderOffsetRad =   -1.0295;//(2 * Math.PI) - 2.112 - 2.104;//(2 * Math.PI) -.19;//5.4681;// Units.degreesToRadians(313.3);  //5.44;
    public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 5.1173;// Units.degreesToRadians(293.2);  //5.1250;
    public static final double kBackRightDriveAbsoluteEncoderOffsetRad =  4.2272; // Units.degreesToRadians(242.2);  //4.2092;
  

    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;//was 4/was 2//was 1.2
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 5;//was 4
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;//was .5
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;//was .5
    public static final double kTeleIntakeMaxAccelerationUnitsPerSecond = 5;
    public static final double kTeleShooterMaxAccelerationUnitsPerSecond = 5;

  }
  
  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 2.5;//was 4 was 5
    public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 5;//was 10 was 5
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;//was 3;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 2;//was 10
    public static final double kPXController = .15;//was 1.5
    public static final double kPYController = .15;//was 1.5
    public static final double kPThetaController = .2;//was 3

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = 
            new TrapezoidProfile.Constraints(
                    kMaxAngularSpeedRadiansPerSecond,
                    kMaxAngularAccelerationRadiansPerSecondSquared);
}

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kRobotMechControllerPort = 1;

    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 2;
    public static final int kDriverFieldOrientedButtonIdx = 7;

    public static final int kIntakeAxis = 2;
    public static final int kShooterAxis = 3;


    public static final double kDeadband = 0.1;
  }
}
