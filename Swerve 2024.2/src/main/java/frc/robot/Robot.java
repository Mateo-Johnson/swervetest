// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private RobotContainer robotContainer;
  private static final String defaultAuto = "Default Auto";
  private static final String customAuto1 = "My Auto 1";
  private static final String customAuto2 = "My Auto 2";
  private static final String customAuto3 = "My Auto 3";
  private static final String customAuto4 = "My Auto 4";
  private static final String customAuto5 = "My Auto 5";
  private static final String customAuto6 = "My Auto 6";
  private String autoSelected;
  private final SendableChooser<String> chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();

    for (int port = 5800; port <= 5807; port++) {
      PortForwarder.add(port, "limelight.local", port);
  }

    chooser.setDefaultOption("Default Auto", defaultAuto);
    chooser.addOption("My Auto 1", customAuto1);
    chooser.addOption("My Auto 2", customAuto2);
    chooser.addOption("My Auto 3", customAuto3);
    chooser.addOption("My Auto 4", customAuto4);
    chooser.addOption("My Auto 5", customAuto5);
    chooser.addOption("My Auto 6", customAuto6);
    SmartDashboard.putData("Auto Choices", chooser);
    
  }

 
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autoSelected = chooser.getSelected();
    System.out.println("Auto Selected: " + autoSelected);
  }





  
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (autoSelected) {
      case customAuto1:
      //AUTO 1 CODE AND SHIT
        break;
      case customAuto2:
      //AUTO 2 CODE AND SHIT
        break;
      case customAuto3:
      //AUTO 3 CODE AND SHIT
        break;
      case customAuto4:
      //AUTO 4 CODE AND SHIT
        break;
      case customAuto5:
      //AUTO 5 CODE AND SHIT
        break;
      case customAuto6:
      //AUTO 6 CODE AND SHIT
        break;

        //DEFAULT AUTO (RUNS WHEN YOU FORGET TO SELECT AN AUTO)
      case defaultAuto:
      default:
      //DEFAULT AUTO CODE AND SHIT
        break;
    }

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
