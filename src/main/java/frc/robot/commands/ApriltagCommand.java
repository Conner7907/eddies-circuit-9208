// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.RobotMechSubsystem;
import frc.robot.RobotContainer;

public class ApriltagCommand extends Command {

  private final LimelightSubsystem limelightSubsystem;

  public ApriltagCommand(LimelightSubsystem limelightSubsystem) {
    this.limelightSubsystem = limelightSubsystem;
    addRequirements(limelightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println(limelightSubsystem.getTx());
    if ((limelightSubsystem.getTv() == true) && ((limelightSubsystem.getTx() > -10.0) && (limelightSubsystem.getTx() < 10.0)) && ((limelightSubsystem.getTa() > 4 && (limelightSubsystem.getTa() < 10)))){
      //System.out.println(limelightSubsystem.getTv()+" ApriltagCommand");
      limelightSubsystem.setPurpleLightsOn();
    }else {
      limelightSubsystem.setPurpleLightsOff();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
