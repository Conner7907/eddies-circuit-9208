package frc.robot.subsystems;

//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
//import frc.robot.commands.SpeakerShooterCmd;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

//import edu.wpi.first.wpilibj2.command.WaitUntilCommand; //e's code

import com.revrobotics.RelativeEncoder;

/* 
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.RobotContainer;
*/
public class RobotMechSubsystem extends SubsystemBase {
    //create solenoids and compressor
    public final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    public final Solenoid AmpSolenoid = new Solenoid(24, PneumaticsModuleType.CTREPCM, 0);



    public final Solenoid OrangeLight = new Solenoid(24, PneumaticsModuleType.CTREPCM, 7);
   // public final Solenoid ClimbSolenoid = new Solenoid(13, PneumaticsModuleType.CTREPCM, 1);

    Timer timer = new Timer();

    //public final DoubleSolenoid ClimbingLockSolenoid = new DoubleSolenoid(13, PneumaticsModuleType.CTREPCM, 1, 2);

   

    //creates intake and shooting motors
    public CANSparkMax Intake550 = new CANSparkMax(14, MotorType.kBrushless);
    //public CANSparkMax ShooterMotor = new CANSparkMax(15, MotorType.kBrushless); //not spark
    public VictorSPX shooterLower = new VictorSPX(23);
    public VictorSPX shooterUpper = new VictorSPX(21);
    public VictorSPX ampRoller = new VictorSPX(22);
    //public CANSparkMax ClimbingMotorRight = new CANSparkMax(16, MotorType.kBrushless);
    //public CANSparkMax ClimbingMotorLeft = new CANSparkMax(17, MotorType.kBrushless);

    public VictorSPX rightIntakeDC = new VictorSPX(18);
    public VictorSPX leftIntakeDC = new VictorSPX(19);


    //Climbing relative encoders
    //public RelativeEncoder ShooterEncoder = ShooterMotor.getEncoder();
    //public RelativeEncoder ClimbLeftEncoder = ClimbingMotorLeft.getEncoder();
    public final DigitalInput digitalInputIntake = new DigitalInput(3);// 
    //public boolean intakeLimitSwitch1 = digitalInputIntake.get();

    


    
    public double time;
    public double elapsedTime;
    public boolean intakeSwitch;
    public int i;

    public RobotMechSubsystem(){
      Intake550.setInverted(true);
      shooterUpper.setInverted(true);
      shooterLower.setInverted(true);
      ampRoller.setInverted(true);
      rightIntakeDC.setInverted(true);
      leftIntakeDC.setInverted(false);
      
      //shooterUpper.setIdleMode(IdleMode.kBrake);
      //Intake550.setIdleMode(IdleMode.kBrake);
    }
    
    public void setIntake(double speed550, double speedDC){
      Intake550.set(speed550);
      rightIntakeDC.set(ControlMode.PercentOutput, speedDC);
      leftIntakeDC.set(ControlMode.PercentOutput, speedDC);
    }

    public void setShooterUpper(double speedUpper){
      shooterUpper.set(ControlMode.PercentOutput, speedUpper);
    }

    public void setShooterLower(double speedLower){
      shooterLower.set(ControlMode.PercentOutput, speedLower);
    }

    public void ampRoller(double speedRoller){
      ampRoller.set(ControlMode.PercentOutput, speedRoller);
    }

    //fuction to open solenoid
    public void setAmpSolenoidOn(){
      AmpSolenoid.set(true);
    }
    //function to close solenoid
    public void setAmpSolenoidOff(){
      AmpSolenoid.set(false);
    }



    public boolean getIntakeSensor(){
      return digitalInputIntake.get();
    }

    public boolean getSourceIntakeSensor(){
      return digitalInputIntake.get();
    }

    public void setLightsOn(){
     OrangeLight.set(true);
    }

    public void setLightsOff(){
     OrangeLight.set(false);
    }

    public void countSourceIntake(){
      i++;
    }
    
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      //SmartDashboard.putNumber("Shooter Velocity", ShooterEncoder.getVelocity());
      SmartDashboard.putBoolean("Intake Sensor", digitalInputIntake.get());
      
    }
  }
    
  
