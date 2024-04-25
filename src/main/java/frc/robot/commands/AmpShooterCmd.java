// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotMechSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class AmpShooterCmd extends Command {

public final RobotMechSubsystem robotMechSubsystem;

double startTime;

public AmpShooterCmd(RobotMechSubsystem robotMechSubsystem) {
  this.robotMechSubsystem = robotMechSubsystem;
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
    double elapsedTime = Timer.getFPGATimestamp() - startTime;

    robotMechSubsystem.setAmpSolenoidOn();
    robotMechSubsystem.ampRoller(.75);

    if (elapsedTime > 1.0){
      robotMechSubsystem.setIntake(1, 0);
      robotMechSubsystem.setShooterUpper(.30);
      robotMechSubsystem.setShooterLower(.25);
    }else{
      robotMechSubsystem.setIntake(0, 0);
      robotMechSubsystem.setShooterUpper(0);
      robotMechSubsystem.setShooterLower(0);
    }

    if (robotMechSubsystem.getIntakeSensor() == false){
      robotMechSubsystem.setLightsOn();
    }else {
      robotMechSubsystem.setLightsOff();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      robotMechSubsystem.setIntake(0, 0);
      robotMechSubsystem.setShooterLower(0);
      robotMechSubsystem.setShooterUpper(0);
      robotMechSubsystem.ampRoller(0);
      robotMechSubsystem.setAmpSolenoidOff();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
