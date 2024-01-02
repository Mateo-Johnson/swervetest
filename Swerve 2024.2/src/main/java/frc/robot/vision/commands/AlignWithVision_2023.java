// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightLib;

public class AlignWithVision_2023 extends CommandBase {
  /** Creates a new AlignWithVision_2023. */


  public static String limelightName = "limelight";

  double tX = LimelightLib.getTX(limelightName);
  double tY = LimelightLib.getTY(limelightName);
  double tA = LimelightLib.getTA(limelightName);
  boolean tV = LimelightLib.getTV(limelightName);
  Pose3d robotPose3d = LimelightLib.getBotPose3d(limelightName);
  Pose3d targetPose3d = LimelightLib.getTargetPose3d_RobotSpace(limelightName);
  double targetID = LimelightLib.getFiducialID(limelightName);

  public AlignWithVision_2023() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    if (tV == true) {
      float kP = -0.1f;

      double steeringAdjust = kP * tX;

    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
