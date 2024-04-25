// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotMechSubsystem;

public class AmpShooterCmd2 extends Command {

public final RobotMechSubsystem robotMechSubsystem;

public AmpShooterCmd2(RobotMechSubsystem robotMechSubsystem) {
  this.robotMechSubsystem = robotMechSubsystem;
  addRequirements(robotMechSubsystem);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*robotMechSubsystem.setAmpSolenoidOn();
    robotMechSubsystem.setShooter(.25);//was .23
    
    if(robotMechSubsystem.getShooterVelocity() > 900){//was 1000
      robotMechSubsystem.setIntake(1, 0);
    }else if(robotMechSubsystem.getShooterVelocity() <= 900){//was 900
      robotMechSubsystem.setIntake(0, 0);
    }*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      robotMechSubsystem.setIntake(0, 0);
      //robotMechSubsystem.setShooter(0);
      robotMechSubsystem.setAmpSolenoidOff();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
