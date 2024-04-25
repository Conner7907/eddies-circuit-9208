// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotMechSubsystem;

public class IntakeCmd extends Command {

  /** Creates a new IntakeCmd. */

  private final RobotMechSubsystem robotMechSubsystem;

  public IntakeCmd(RobotMechSubsystem robotMechSubsystem) {
    this.robotMechSubsystem = robotMechSubsystem;
    addRequirements(robotMechSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(robotMechSubsystem.getIntakeSensor());
    if(robotMechSubsystem.getIntakeSensor() == true){
      robotMechSubsystem.setIntake(.4,.5);
    }else if(robotMechSubsystem.getIntakeSensor() == false){
      robotMechSubsystem.setIntake(0,0);    
      robotMechSubsystem.setLightsOn();
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
    return false;
  }
}
