// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;


public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem robotDrive = new DriveSubsystem();

  //DRIVER CONTROLLERS
  CommandXboxController primaryDriver = new CommandXboxController(0);
  CommandXboxController secondaryDriver = new CommandXboxController(0);


  public RobotContainer() {



    configureButtonBindings(); //CONFIGURE BINDINGS

    // Configure default commands
    robotDrive.setDefaultCommand(

        new RunCommand(
            () -> robotDrive.drive(
                //LEFT STICK IS TRANSLATION RIGHT STICK IS TURNING
                -MathUtil.applyDeadband(primaryDriver.getLeftY(), ControllerConstants.driveDeadzone),
                -MathUtil.applyDeadband(primaryDriver.getLeftX(), ControllerConstants.driveDeadzone),
                -MathUtil.applyDeadband(primaryDriver.getRightX(), ControllerConstants.driveDeadzone),
                true, true),
            robotDrive));
  }


  private void configureButtonBindings() {
    //DEFINE ALL OF THE BUTTON BINDINGS HERE PLEASE AND THANKS
    primaryDriver.rightBumper()
        .whileTrue(new RunCommand(
            () -> robotDrive.setX(),
            robotDrive));
            
  }

  //THIS IS ALL OF THE AUTO SHIT PLEASE DON'T WRITE AUTO SHIT ANYWHERE ELSE
  public Command getAutonomousCommand() {

    return new PathPlannerAuto("New Auto");

    // // Create config for trajectory
    // TrajectoryConfig config = new TrajectoryConfig(
    //     AutoConstants.maxSpeedMetersPerSecond,
    //     AutoConstants.maxAccelerationMetersPerSecondSquared)
    //     // Add kinematics to ensure max speed is actually obeyed
    //     .setKinematics(DriveConstants.DriveKinematics);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(3, 0, new Rotation2d(0)),
    //     config);

    // var thetaController = new ProfiledPIDController(
    //     AutoConstants.PThetaController, 0, 0, AutoConstants.thetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //     exampleTrajectory,
    //     robotDrive::getPose, // Functional interface to feed supplier
    //     DriveConstants.DriveKinematics,

    //     // Position controllers
    //     new PIDController(AutoConstants.PXController, 0, 0),
    //     new PIDController(AutoConstants.PYController, 0, 0),
    //     thetaController,
    //     robotDrive::setModuleStates,
    //     robotDrive);

    // // Reset odometry to the starting pose of the trajectory.
    // robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> robotDrive.drive(0, 0, 0, false, false));
  }
}
