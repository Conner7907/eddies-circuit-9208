// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RobotMechSubsystem;

public class SpeakerShooterCmd extends Command {
  /** Creates a new SpeakerShooterCmd. */
  private final RobotMechSubsystem robotMechSubsystem;
  public SpeakerShooterCmd(RobotMechSubsystem robotMechSubsystem) {
    this.robotMechSubsystem = robotMechSubsystem;
    addRequirements(robotMechSubsystem);
  }

  double startTimer;
  double elapsedTime;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTimer = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elapsedTime = Timer.getFPGATimestamp() - startTimer;
    robotMechSubsystem.setShooterUpper(.85); //was 85
    robotMechSubsystem.setShooterLower(.60); //was 60
    if (elapsedTime > 1){
      robotMechSubsystem.setIntake(1, 0);
    }else{
      robotMechSubsystem.setIntake(0, 0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    robotMechSubsystem.setIntake(0, 0);
    robotMechSubsystem.setShooterUpper(0);
    robotMechSubsystem.setShooterLower(0);
    robotMechSubsystem.setLightsOff();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
