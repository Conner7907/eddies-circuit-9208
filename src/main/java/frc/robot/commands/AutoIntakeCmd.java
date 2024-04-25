// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.RobotMechSubsystem;
//import edu.wpi.first.wpilibj.Timer;

public class AutoIntakeCmd extends Command {
  double startTime;
  double elapsedTime;
  private final RobotMechSubsystem robotMechSubsystem;
  /** Creates a new AutoIntakeCmd. */
  public AutoIntakeCmd(RobotMechSubsystem robotMechSubsystem) {
    this.robotMechSubsystem = robotMechSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(robotMechSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elapsedTime = Timer.getFPGATimestamp() - startTime;
    if(robotMechSubsystem.getIntakeSensor()){
      robotMechSubsystem.setIntake(.4,.4);

    }
    else if(!robotMechSubsystem.getIntakeSensor()){
      robotMechSubsystem.setIntake(0,0);
    }
  }    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      robotMechSubsystem.setIntake(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(elapsedTime);
    if(!robotMechSubsystem.getIntakeSensor()||elapsedTime > 5){
      return true;
    }else{
      return false;
    }
  }
}
