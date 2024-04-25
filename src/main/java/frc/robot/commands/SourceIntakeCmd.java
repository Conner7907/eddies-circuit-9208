// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;

//import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.RobotMechSubsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SourceIntakeCmd extends Command {


  public final RobotMechSubsystem robotMechSubsystem;
  //public final Supplier<Boolean> SourceSensor;
  public Trigger intakeTrigger;
  public boolean SourceIntake;
  
  
  public SourceIntakeCmd(RobotMechSubsystem robotMechSubsystem) {
    this.robotMechSubsystem = robotMechSubsystem;
  
    addRequirements(robotMechSubsystem);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
  //Trigger intakeTrigger = new Trigger(robotMechSubsystem.digitalInputIntake::get);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

      double sensorAnalog = 0;
      robotMechSubsystem.setIntake(-.5, 0);//Intake 550 was -.5
      robotMechSubsystem.setShooterUpper(-.5);
      robotMechSubsystem.setShooterLower(-.5);
      if (robotMechSubsystem.getIntakeSensor() == true){
        sensorAnalog = 1.0;
      }else {
        sensorAnalog = 0;
      }
      // Creates a new high-pass IIR filter
      // Time constant is 0.1 seconds
      // Period is 0.02 seconds - this is the standard FRC main loop period
      LinearFilter filter = LinearFilter.highPass(0.1, 0.02);
      filter.calculate(sensorAnalog);

      System.out.println(filter.calculate(sensorAnalog));
      /*do{
        intakeTrigger.onFalse(new RunCommand(() -> robotMechSubsystem.countSourceIntake()));
        if(robotMechSubsystem.i == 2){

          robotMechSubsystem.setIntake(0, 0);//Intake 550 was -.5
          robotMechSubsystem.setShooter(0);
          SourceIntake = false;

        }

      }while(SourceIntake);
      robotMechSubsystem.i=0;

    */
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    robotMechSubsystem.setIntake(0, 0);
    robotMechSubsystem.setShooterUpper(0);
    robotMechSubsystem.setShooterLower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
