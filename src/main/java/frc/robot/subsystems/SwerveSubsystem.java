package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;


/* 
Currently unused imports
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
*/
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Pose2d;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;


public class SwerveSubsystem extends SubsystemBase{
    
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftDriveEncoderReversed,
        DriveConstants.kFrontLeftTurningEncoderReversed,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kFrontRightDriveEncoderReversed,
        DriveConstants.kFrontRightTurningEncoderReversed,
        DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kBackLeftDriveEncoderReversed,
        DriveConstants.kBackLeftTurningEncoderReversed,
        DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightTurningMotorPort,
        DriveConstants.kBackRightDriveEncoderReversed,
        DriveConstants.kBackRightTurningEncoderReversed,
        DriveConstants.kBackRightDriveAbsoluteEncoderPort,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kBackRightDriveAbsoluteEncoderReversed);


    private final AHRS gyro = new AHRS();

    private PowerDistribution m_PDPanel = new PowerDistribution(1, ModuleType.kRev);

/*try {
    gyro = new AHRS(); //SPI.Port.kMXP
  } catch (RuntimeException ex) {
    DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
  }*/


    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, new Rotation2d(0), new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
    } );


    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();

              AutoBuilder.configureHolonomic(
                      this::getPose, // Robot pose supplier
                      this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                      this::pathGetSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                      this::pathDriveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                              new PIDConstants(.15, 0.0, 0.0), // Translation PID constants kP was 5
                              new PIDConstants(.2, 0.0, 0.0), // Rotation PID constants kP was 5
                              4.25, // Max module speed, in m/s
                              0.43, // Drive base radius in meters. Distance from robot center to furthest module.
                              new ReplanningConfig() // Default path replanning config. See the API for the options here
                      ),
                      () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                          return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                      },
                      this // Reference to this subsystem to set requirements
              );
          
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(-gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());//confirm Rotation 2d is adequate
    }


    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public ChassisSpeeds pathGetSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState()
        );
    }


    public void resetOdometry(Pose2d pose) {    
        odometer.resetPosition(getRotation2d(), new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()}
            ,pose);
    }


    @Override
    public void periodic() {
        odometer.update(getRotation2d(), new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });
        
        
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putNumber("Non-zeroed Yaw",gyro.getYaw());
        SmartDashboard.putNumber("Non-zeroed Pitch",gyro.getPitch());
        SmartDashboard.putNumber("Non-zeroed Roll",gyro.getRoll());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        
        // Get the voltage going into the PDP, in Volts.
        // The PDP returns the voltage in increments of 0.05 Volts.
        double voltage = m_PDPanel.getVoltage();
        SmartDashboard.putNumber("Voltage", voltage);

        // Get the total current of all channels.
        double totalCurrent = m_PDPanel.getTotalCurrent();
        SmartDashboard.putNumber("Total Current", totalCurrent);

        // Get the current going through channel X, in Amperes.
        // The PDP returns the current in increments of 0.125A.
        // At low currents the current readings tend to be less accurate.
        //double current0 = m_PDPanel.getCurrent(0);
        //SmartDashboard.putNumber("Current Channel 0", current0);
        
        //double current1 = m_PDPanel.getCurrent(1);
        //SmartDashboard.putNumber("Current Channel 1", current1);
        
        //double current2 = m_PDPanel.getCurrent(2);
        //SmartDashboard.putNumber("Current Channel 2", current2);
        
        //double current3 = m_PDPanel.getCurrent(3);
        //SmartDashboard.putNumber("Current Channel 3", current3);
        //
        double current4 = m_PDPanel.getCurrent(4);
        SmartDashboard.putNumber("Shooter Current Channel", current4);
        SmartDashboard.putNumber("Shooter Power", voltage*current4);

        
        //double current5 = m_PDPanel.getCurrent(5);
        //SmartDashboard.putNumber("Current Channel 5", current5);
        
        //double current6 = m_PDPanel.getCurrent(6);
        //SmartDashboard.putNumber("Current Channel 6", current6);
        
        //double current7 = m_PDPanel.getCurrent(7);
        //SmartDashboard.putNumber("Current Channel 7", current7);
        
        //double current8 = m_PDPanel.getCurrent(8);
        //SmartDashboard.putNumber("Current Channel 8", current8);
        //
        //double current9 = m_PDPanel.getCurrent(9);
        //SmartDashboard.putNumber("Current Channel 9", current9);
        //
        //double current10 = m_PDPanel.getCurrent(10);
        //SmartDashboard.putNumber("Current Channel 10", current10);
        //
        //double current11 = m_PDPanel.getCurrent(11);
        //SmartDashboard.putNumber("Current Channel 11", current11);
        //
        //double current12 = m_PDPanel.getCurrent(12);
        //SmartDashboard.putNumber("Current Channel 12", current12);
        //
        //double current13 = m_PDPanel.getCurrent(13);
        //SmartDashboard.putNumber("Current Channel 13", current13);
        //
        //double current14 = m_PDPanel.getCurrent(14);
        //SmartDashboard.putNumber("Current Channel 14", current14);
        //
        //double current16 = m_PDPanel.getCurrent(16);
        //SmartDashboard.putNumber("Current Channel 16", current16);

        //double current17 = m_PDPanel.getCurrent(17);
        //SmartDashboard.putNumber("Current Channel 17", current17);

        //double current18 = m_PDPanel.getCurrent(18);
        //SmartDashboard.putNumber("Current Channel 18", current18);

        //double current19 = m_PDPanel.getCurrent(19);
        //SmartDashboard.putNumber("Current Channel 19", current19);


    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void pathDriveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond); // normalize wheel speed
        frontLeft.setDesiredState(desiredStates[2]);
        frontRight.setDesiredState(desiredStates[3]);
        backLeft.setDesiredState(desiredStates[0]);
        backRight.setDesiredState(desiredStates[1]);
    }


    

    public void getAbsoluteEncoderPosition(){
        SmartDashboard.putNumber("Front Left Absolute Encoder", frontLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Front Right Absoluet Encoder", frontRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Back Left Absolute Encoder", backLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Back Right Absolute Encoder", backRight.getAbsoluteEncoderRad());
    }

    

}
