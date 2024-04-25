// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Robot;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.RobotMechSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;
import edu.wpi.first.math.util.Units;
//import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
//import frc.robot.commands.RobotMechJoystickCmd;
//import edu.wpi.first.wpilibj.DigitalInput;  
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

//import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.Timer;



//Currently unused imports

//import edu.wpi.first.wpilibj.Timer;
//import frc.robot.Constants.OperatorConstants;
//import edu.wpi.first.wpilibj2.command.Comma;




/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final RobotMechSubsystem robotMechSubsystem = new RobotMechSubsystem();
  private final LiftSubsystem liftSubsystem = new LiftSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Joystick TwistJoystick = new Joystick(OIConstants.kDriverControllerPort);
  private final Joystick DriveJoystick = new Joystick(2);

  public final static XboxController RobotMechJoystick = new XboxController(OIConstants.kRobotMechControllerPort);

  private final SendableChooser<Command> m_chooser;


  //public final DigitalInput digitalInputIntake = new DigitalInput(0);  
  //public final DigitalInput RightLimitSwitch = new DigitalInput(1);  -CBtest
  //public final DigitalInput LeftLimitSwitch = new DigitalInput(2);  -CDtest




  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {


  //Register Named Commands
  NamedCommands.registerCommand("AutoIntakeCmd", new AutoIntakeCmd(robotMechSubsystem));
  NamedCommands.registerCommand("AutoShooterCmd", new AutoShooterCmd(robotMechSubsystem));
  NamedCommands.registerCommand("AutoShooterCmd2", new AutoShooterCmd2(robotMechSubsystem));


    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> -DriveJoystick.getRawAxis(1),//OIConstants.kDriverYAxis
      () -> -DriveJoystick.getRawAxis(0),//OIConstants.kDriverXAxis
      () -> TwistJoystick.getRawAxis(2),//OIConstants.kDriverRotAxis
      () -> !TwistJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)
      ));

    liftSubsystem.setDefaultCommand(new LiftJoystickCommand(
      liftSubsystem,
      () -> RobotMechJoystick.getRawAxis(1),
      () -> RobotMechJoystick.getRawAxis(5),
      () -> RobotMechJoystick.getRawButton(7)
      ));

    configureButtonBindings();
    
    m_chooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", m_chooser);
    limelightSubsystem.setDefaultCommand(new ApriltagCommand(limelightSubsystem));

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureButtonBindings() { 
        //resets yaw/Heading when button 1 is pressed
        new JoystickButton(TwistJoystick, 3).whileTrue(new RunCommand(
                () ->swerveSubsystem.zeroHeading()));

        new JoystickButton(RobotMechJoystick,5).whileTrue(new IntakeCmd(robotMechSubsystem));
        new JoystickButton(RobotMechJoystick,1).whileTrue(new SpeakerShooterCmd(robotMechSubsystem));
        new JoystickButton(RobotMechJoystick,3).whileTrue(new AmpShooterCmd(robotMechSubsystem));
        new JoystickButton(RobotMechJoystick,4).whileTrue(new SourceIntakeCmd(robotMechSubsystem));
        new JoystickButton(RobotMechJoystick,6).whileTrue(new RunCommand(() -> swerveSubsystem.getAbsoluteEncoderPosition()));
  }


  
  /* public void Update_Limelight_Tracking(){

        // These numbers must be tuned for your Robot!  Be careful!
        /* 
        
        final double STEER_K = 0.03;                    // how hard to turn toward the target
        final double DRIVE_K = 0.26;                    // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 13.0;        // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast
        
        boolean m_LimelightHasValidTarget;
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

        SmartDashboard.putNumber("tv", tv);
        SmartDashboard.putNumber("tx", tx);
        SmartDashboard.putNumber("ty", ty);
        SmartDashboard.putNumber("ta", ta);

        if (tv < 1.0)
        {
          m_LimelightHasValidTarget = false;
          //m_LimelightDriveCommand = 0.0;
          //m_LimelightSteerCommand = 0.0;
          //return;
        }

        m_LimelightHasValidTarget = true;

        /* 
        // Start with proportional steering
        double steer_cmd = tx * STEER_K;
        m_LimelightSteerCommand = steer_cmd;

        // try to drive forward until the target area reaches our desired area
        double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

        // don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_DRIVE)
        {
          drive_cmd = MAX_DRIVE;
        }
        m_LimelightDriveCommand = drive_cmd;
  } */

/*
  public void AutonInit(){
        m_chooser.setDefaultOption("Default Auto", kNothingAuto);
        //m_chooser.addOption("My Auto", kCustomAuto);
        m_chooser.addOption("Blue Amp / Red Source Side Auto", kAmpSide);
        m_chooser.addOption("Center Side Auto", kCenter);
        m_chooser.addOption("Blue Source / Red Amp Side Auto", kSource);
        
        SmartDashboard.putData("Auto choices",m_chooser);
  }

  public void AutonSelector(){

        m_autoSelected = m_chooser.getSelected();
        System.out.println("Auto selected:" + m_autoSelected);
        switch (m_autoSelected) {
          case kNothingAuto:
    
            break;
          case kAmpSide:
            m_autonomousCommand = getAutonomousCommandAmpPath();
                 // schedule the autonomous command (example)
            if (m_autonomousCommand != null) {
            System.out.println("auto init");
            m_autonomousCommand.schedule();
            }
            break;
          case kCenter:
            m_autonomousCommand = getAutonomousCommandCenter();
                 // schedule the autonomous command (example)
            if (m_autonomousCommand != null) {
            System.out.println("auto init");
            m_autonomousCommand.schedule();
            }
            break;
          case kSource:
            m_autonomousCommand = getAutonomousCommandSource();
                 // schedule the autonomous command (example)
            if (m_autonomousCommand != null) {
            System.out.println("auto init");
            m_autonomousCommand.schedule();
            }
            break;
    
          default:
            break;
        }
  }
*/


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommandAmpPath() {
        return new PathPlannerAuto("BlueAmpAuto");
      }


/*
  public Command getAutonomousCommandAmp() {
        // 1. Create trajectory settings
        System.out.println("getAutonomous BlueAmp/RedSource");
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory
        Trajectory trajectory0 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)), //Initial point
                List.of(
                        new Translation2d(.5,.5)
                        //new Translation2d(-.5,.5)
                ),
                new Pose2d(1, 1, Rotation2d.fromDegrees(90)), //final point
                trajectoryConfig);   //trajectory config

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)), //Initial point

                List.of(

                ),
                new Pose2d(-2, 1.8, Rotation2d.fromDegrees(61.68)), //final point
                trajectoryConfig);   //trajectory config

        Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(-2, 1.8, new Rotation2d(1)), //Initial point
                List.of(
                        new Translation2d(-1.8,1.6),
                        new Translation2d(-.5,.5)
                ),
                new Pose2d(0, 0, Rotation2d.fromDegrees(-61.68 * 2)), //final point
                trajectoryConfig);   //trajectory config


        
        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(.4, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand0 = new SwerveControllerCommand(
                Robot.trajectoryPPP0,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem); 

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem); 

        SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(
                trajectory2,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem); 

       

        // 5. Add some init and wrap-up, and return everything

       
        return new SequentialCommandGroup(
               // new InstantCommand(() -> swerveSubsystem.zeroHeading()),
                new InstantCommand(() -> swerveSubsystem.resetOdometry(Robot.trajectoryPPP0.getInitialPose())),
                //new AutoShooterCmd(robotMechSubsystem),
                swerveControllerCommand0, //need to test 
                /*new ParallelCommandGroup( 
                        swerveControllerCommand, 
                        new SequentialCommandGroup(
                                new WaitCommand(1),
                                new AutoIntakeCmd(robotMechSubsystem))),
               swerveControllerCommand2,
               //new AutoShooterCmd(robotMechSubsystem),
                new InstantCommand(() -> swerveSubsystem.stopModules()));
  }
*/


/*
    public Command getAutonomousCommandSource() {
        // 1. Create trajectory settings
        System.out.println("getAutonomous BlueSource/RedAmp");
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                1.5,//AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory
        Trajectory trajectory3 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)), //Initial point
                List.of(

                ),
                new Pose2d(-1.8, -1.6, Rotation2d.fromDegrees(-70)), //final point
                trajectoryConfig);   //trajectory config

        Trajectory trajectory4 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(-1.8, -1.6, new Rotation2d(Units.degreesToRadians(-70))),//Units.degreesToRadians(-45))), //Initial point
                List.of(
                        new Translation2d(-1.5, -1.4),
                        new Translation2d(-.3, 0)
                ),
                new Pose2d(0, 0, Rotation2d.fromDegrees(100)), //final point
                trajectoryConfig);   //trajectory config


        Trajectory trajectory30 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(.73, 6.67, new Rotation2d(Units.degreesToRadians(21.7))), //Initial point
                List.of(

                ),
                new Pose2d(2.56, 6.97, Rotation2d.fromDegrees(0)), //final point
                trajectoryConfig);   //trajectory config

        Trajectory trajectory40 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(2.65, 6.97, new Rotation2d(Units.degreesToRadians(0))), //Initial point
                List.of(
                        //new Translation2d(-1.7, -1.6),
                        //new Translation2d(-.3, 0)
                ),
                new Pose2d(.73, 6.67, Rotation2d.fromDegrees(21.7)), //final point
                trajectoryConfig);   //trajectory config

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(/*AutoConstants.kPThetaController.3, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand3 = new SwerveControllerCommand(
                trajectory3,//was 3
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem); 

        SwerveControllerCommand swerveControllerCommand4 = new SwerveControllerCommand(
                trajectory4,//was 4
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem); 

       

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.zeroHeading()),
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory3.getInitialPose())),
                new AutoShooterCmd(robotMechSubsystem),
                new ParallelCommandGroup(
                        swerveControllerCommand3,
                        new SequentialCommandGroup(
                                new WaitCommand(1),
                                new AutoIntakeCmd(robotMechSubsystem))),
                swerveControllerCommand4,
                new AutoShooterCmd(robotMechSubsystem),
                new InstantCommand(() -> swerveSubsystem.stopModules()));
  }

*/

   public Command getAutonomousCommandCenter() {
        // 1. Create trajectory settings
        System.out.println("getAutonomous Center");
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);


       // 2. Generate trajectory
        Trajectory trajectory6 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)), //Initial point
                List.of(
                        new Translation2d(-.88, .01)   //way point
                ),
                new Pose2d(-1.76, .01, Rotation2d.fromDegrees(0)), //final point
                trajectoryConfig);   //trajectory config

        Trajectory trajectory7 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(-1.76, .01, new Rotation2d(0)), //Initial point
                List.of(
                        new Translation2d(-.88, 0.01)   //way point
                ),
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)), //final point
                trajectoryConfig);   //trajectory config



        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory

        SwerveControllerCommand swerveControllerCommand5 = new SwerveControllerCommand(
                //trajectory,
                trajectory6,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);
                
        SwerveControllerCommand swerveControllerCommand6 = new SwerveControllerCommand(
                //Robot.trajectoryPW1,
                trajectory7,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);
                
                


        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
               new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory6.getInitialPose())),
                new AutoShooterCmd(robotMechSubsystem),
                new ParallelCommandGroup( 
                        swerveControllerCommand5, 
                        new SequentialCommandGroup(
                                new WaitCommand(1),
                                new AutoIntakeCmd(robotMechSubsystem))),
                swerveControllerCommand6,
                new AutoShooterCmd(robotMechSubsystem),
                new InstantCommand(() -> swerveSubsystem.stopModules()));
  }
}
          