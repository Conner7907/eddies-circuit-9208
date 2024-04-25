// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
//import frc.robot.RobotContainer;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.subsystems.RobotMechSubsystem;

/* Currently unused Imports
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
*/

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
//robot
public class Robot extends TimedRobot {
  
  private Command m_autonomousCommand;

  private static final String kNothingAuto = "Do Nothing";
  private static final String kAmpSide = "Amp Side Auto";
  private static final String kCenter = "Center Side Auto";
  private static final String kSource = "Source Side Auto";

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
  private RobotContainer m_robotContainer;
 

 //Import Path1 JSON center speaker to center note
  //String trajectoryPP0 = "pathplanner/paths/BlueAmp.path";
  //String trajectoryPW1JSON = "paths/CollectToStart.wpilib.json";  
  //public static Trajectory trajectoryPPP0 = new Trajectory();
  //public static Trajectory trajectoryPW1 = new Trajectory();

/* 
  //Import Path2 JSON amp side speaker to amp side note
  String trajectoryPW2JSON = "paths/AmpSideToAmpSideNote.wpilib.json";
  String trajectoryPW3JSON = "paths/AmpSideNoteToAmpSide.wpilib.json";  
  public static Trajectory trajectoryPW2 = new Trajectory();
  public static Trajectory trajectoryPW3 = new Trajectory();


  //Import Path3 JSON source side speaker to source side note
  String trajectoryPW4JSON = "paths/AmpSideToAmpSideNote.wpilib.json";
  String trajectoryPW5JSON = "paths/AmpSideNoteToAmpSide.wpilib.json";  
  public static Trajectory trajectoryPW4 = new Trajectory();
  public static Trajectory trajectoryPW5 = new Trajectory();

*/

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
   
    m_robotContainer = new RobotContainer();
    //m_robotContainer.AutonInit();
  
    
    m_chooser.setDefaultOption("Default Auto", kNothingAuto);
    //m_chooser.addOption("My Auto", kCustomAuto);
    m_chooser.addOption("Blue Amp / Red Source Side Auto", kAmpSide);
    m_chooser.addOption("Center Side Auto", kCenter);
    m_chooser.addOption("Blue Source / Red Amp Side Auto", kSource);
    
    SmartDashboard.putData("Auto choices",m_chooser);
     
    CameraServer.startAutomaticCapture(0);
    CameraServer.startAutomaticCapture(1);
    
/*
      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryPP0);
        trajectoryPPP0 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryPP0, ex.getStackTrace());

      }
*/   
/*
      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryPW1JSON);
        trajectoryPW1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryPW1JSON, ex.getStackTrace());
      }


      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryPW2JSON);
        trajectoryPW2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryPW2JSON, ex.getStackTrace());
      }


      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryPW3JSON);
        trajectoryPW3 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryPW3JSON, ex.getStackTrace());
      }


      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryPW4JSON);
        trajectoryPW4 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryPW4JSON, ex.getStackTrace());
      }


      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryPW5JSON);
        trajectoryPW5 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + trajectoryPW5JSON, ex.getStackTrace());
      }
*/

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
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
    //m_robotContainer.AutonSelector();


    
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected:" + m_autoSelected);
    switch (m_autoSelected) {
      case kNothingAuto:

        break;
      case kAmpSide:
        m_autonomousCommand = m_robotContainer.getAutonomousCommandAmpPath();
             // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
        System.out.println("auto init");
        m_autonomousCommand.schedule();
        }
        break;
      case kCenter:
        m_autonomousCommand = m_robotContainer.getAutonomousCommandCenter();
             // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
        System.out.println("auto init");
        m_autonomousCommand.schedule();
        }
        break;
      case kSource:
        //m_autonomousCommand = m_robotContainer.getAutonomousCommandSource();
             // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
        System.out.println("auto init");
        m_autonomousCommand.schedule();
        }
        break;

      default:
        break;
    }
   
    /*
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      System.out.println("auto init");
      m_autonomousCommand.schedule();
    }
    */

    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
