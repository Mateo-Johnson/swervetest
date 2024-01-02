// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double maxSpeedMetersPerSecond = 4.8;
    public static final double maxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double directionSlewRate = 1.2; // radians per second
    public static final double magnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double rotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double trackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double wheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics DriveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2, trackWidth / 2),
        new Translation2d(wheelBase / 2, -trackWidth / 2),
        new Translation2d(-wheelBase / 2, trackWidth / 2),
        new Translation2d(-wheelBase / 2, -trackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double frontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double frontRightChassisAngularOffset = 0;
    public static final double backLeftChassisAngularOffset = Math.PI;
    public static final double backRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int frontLeftDrivingCanId = 11;
    public static final int rearLeftDrivingCanId = 13;
    public static final int frontRightDrivingCanId = 15;
    public static final int rearRightDrivingCanId = 17;

    public static final int frontLeftTurningCanId = 10;
    public static final int rearLeftTurningCanId = 12;
    public static final int frontRightTurningCanId = 14;
    public static final int rearRightTurningCanId = 16;

    public static final boolean gyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int drivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean turningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double drivingMotorFreeSpeedRps = NeoMotorConstants.freeSpeedRpm / 60;
    public static final double wheelDiameterMeters = 0.0762;
    public static final double wheelCircumferenceMeters = wheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double drivingMotorReduction = (45.0 * 22) / (drivingMotorPinionTeeth * 15);
    public static final double driveWheelFreeSpeedRps = (drivingMotorFreeSpeedRps * wheelCircumferenceMeters)
        / drivingMotorReduction;

    public static final double drivingEncoderPositionFactor = (wheelDiameterMeters * Math.PI)
        / drivingMotorReduction; // meters
    public static final double drivingEncoderVelocityFactor = ((wheelDiameterMeters * Math.PI)
        / drivingMotorReduction) / 60.0; // meters per second

    public static final double turningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double turningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double turningEncoderPositionPIDMinInput = 0; // radians
    public static final double turningEncoderPositionPIDMaxInput = turningEncoderPositionFactor; // radians

    public static final double drivingP = 0.04;
    public static final double drivingI = 0;
    public static final double drivingD = 0;
    public static final double drivingFF = 1 / driveWheelFreeSpeedRps;
    public static final double drivingMinOutput = -1;
    public static final double drivingMaxOutput = 1;

    public static final double turningP = 1;
    public static final double turningI = 0;
    public static final double turningD = 0;
    public static final double turningFF = 0;
    public static final double turningMinOutput = -1;
    public static final double turningMaxOutput = 1;

    public static final IdleMode drivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode turningMotorIdleMode = IdleMode.kBrake;

    public static final int drivingMotorCurrentLimit = 50; // amps
    public static final int turningMotorCurrentLimit = 20; // amps
  }

  public static final class ControllerConstants {
    public static final int driverControllerPort = 0;
    public static final double driveDeadzone = 0.05;
    public static final int driverSecondaryPort = 1;
  }

  public static final class AutoConstants {
    public static final double maxSpeedMetersPerSecond = 3;
    public static final double maxAccelerationMetersPerSecondSquared = 3;
    public static final double maxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double maxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double PXController = 1;
    public static final double PYController = 1;
    public static final double PThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints thetaControllerConstraints = new TrapezoidProfile.Constraints(
        maxAngularSpeedRadiansPerSecond, maxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double freeSpeedRpm = 5676;
  }
}
