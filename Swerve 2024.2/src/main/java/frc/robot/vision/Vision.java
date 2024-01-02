// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.LimelightLib;

public class Vision extends SubsystemBase {

public static String limelightName = "limelight";

double tX = LimelightLib.getTX(limelightName);
double tY = LimelightLib.getTY(limelightName);
double tA = LimelightLib.getTA(limelightName);
boolean tV = LimelightLib.getTV(limelightName);
Pose3d robotPose3d = LimelightLib.getBotPose3d(limelightName);
Pose3d targetPose3d = LimelightLib.getTargetPose3d_RobotSpace(limelightName);
double targetID = LimelightLib.getFiducialID(limelightName);





Vision() {
  


}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
