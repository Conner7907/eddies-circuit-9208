package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
//import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//imports
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotMechSubsystem;
import java.util.function.Supplier;

//import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants.DriveConstants;
//import frc.robot.Constants.OIConstants;




//Robot Mechanism joystick command
public class RobotMechJoystickCmd extends Command{
    //stuff thats called in robot container
    public final RobotMechSubsystem robotMechSubsystem;
    public final Supplier<Boolean> SpeakerButton, AmpButton, IntakeButton, SourceIntakeButton, OverrideButton;
    public final SlewRateLimiter IntakeSlew, ShooterSlew;



    public RobotMechJoystickCmd(RobotMechSubsystem robotMechSubsystem, Supplier<Boolean> Shooterbutton, 
    Supplier<Boolean> AmpButton, Supplier<Boolean> IntakeButton, Supplier<Boolean> SourceIntakeButton, Supplier<Boolean> OverrideButton
    ){
        this.robotMechSubsystem = robotMechSubsystem;
        this.SpeakerButton = Shooterbutton;
        this.IntakeButton = IntakeButton;
        this.SourceIntakeButton = SourceIntakeButton;
        this.OverrideButton = OverrideButton;
        this.AmpButton = AmpButton;
        this.IntakeSlew = new SlewRateLimiter(DriveConstants.kTeleIntakeMaxAccelerationUnitsPerSecond);
        this.ShooterSlew = new SlewRateLimiter(DriveConstants.kTeleShooterMaxAccelerationUnitsPerSecond);

        addRequirements(robotMechSubsystem);
    }



    @Override
    public void initialize() {
        
    }


    @Override
    public void execute() {
        // 1. Get real-time joystick input (declare variables from Suppliers)
        
        // 2. Apply deadband
        //IntakeValue = Math.abs(IntakeValue) > OIConstants.kDeadband ? IntakeValue : 0.0;
        //ShooterValue = Math.abs(ShooterValue) > OIConstants.kDeadband ? ShooterValue : 0.0;

        // 3. Make the acceleration smoother (SlewRate)
        //IntakeValue = IntakeSlew.calculate(IntakeValue);// * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        //ShooterValue = ShooterSlew.calculate(ShooterValue);// * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    }

    @Override
    public void end(boolean interrupted) {

    }

}
