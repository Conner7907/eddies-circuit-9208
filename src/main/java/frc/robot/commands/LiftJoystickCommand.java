// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import java.sql.RowId;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.LiftSubsystem;

public class LiftJoystickCommand extends Command {
  /** Creates a new LiftCommand. */
  private final LiftSubsystem liftSubsystem;

  public final Supplier<Double> RightClimbAxis, LeftClimbAxis;
  public final Supplier<Boolean> LiftOverride;
  
  
  public LiftJoystickCommand(LiftSubsystem liftSubsystem, Supplier<Double> RightClimbAxis, Supplier<Double> LeftClimbAxis, Supplier<Boolean> LiftOverride) {
    this.liftSubsystem = liftSubsystem;
    this.RightClimbAxis = RightClimbAxis;
    this.LeftClimbAxis = LeftClimbAxis;
    this.LiftOverride = LiftOverride;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(liftSubsystem);
 
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    liftSubsystem.resetClimbEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rightLift = RightClimbAxis.get();
    double leftLift = LeftClimbAxis.get();

    rightLift = Math.abs(rightLift) > OIConstants.kDeadband ? rightLift : 0.0;
    leftLift = Math.abs(leftLift) > OIConstants.kDeadband ? leftLift : 0.0;

    if(LiftOverride.get()){

      liftSubsystem.setRightLift(rightLift);
      liftSubsystem.setLeftLift(leftLift);


      //RightClimbAxisValue = Math.abs(RightClimbAxisValue) > OIConstants.kDeadband ? RightClimbAxisValue : 0.0;
      //LeftClimbAxisValue = Math.abs(LeftClimbAxisValue) > OIConstants.kDeadband ? LeftClimbAxisValue : 0.0;


    }else if(!LiftOverride.get()){
      if((liftSubsystem.getRightClimbEncoder() >= -96) && (liftSubsystem.getRightClimbEncoder() <= 0)&&(!liftSubsystem.getRightLimit())){
          liftSubsystem.setRightLift(rightLift);
      }else if(liftSubsystem.getRightClimbEncoder() < -96){
        if(rightLift > 0){
          liftSubsystem.setRightLift(rightLift);
        }else if(rightLift <= 0){
          liftSubsystem.setRightLift(0);
        }
      }else if(liftSubsystem.getRightClimbEncoder() > 0 || liftSubsystem.getRightLimit()){
        if(rightLift < 0){
          liftSubsystem.setRightLift(rightLift);
        }else if(rightLift >= 0){
          liftSubsystem.setRightLift(0);
        }
      }

      if((liftSubsystem.getLeftClimbEncoder() >= -104) && (liftSubsystem.getLeftClimbEncoder() <= 0)&&(!liftSubsystem.getLeftLimit())){
          liftSubsystem.setLeftLift(leftLift);
      }else if(liftSubsystem.getLeftClimbEncoder() < -104){
        if(leftLift > 0){
          liftSubsystem.setLeftLift(leftLift);
        }else if(leftLift <= 0){
          liftSubsystem.setLeftLift(0);
        }
      }else if(liftSubsystem.getLeftClimbEncoder() > 0 || liftSubsystem.getLeftLimit()){
        if(leftLift < 0){
          liftSubsystem.setLeftLift(leftLift);
        }else if(leftLift >= 0){
        liftSubsystem.setLeftLift(0);
        }
      }
   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    liftSubsystem.setRightLift(0);
    liftSubsystem.setLeftLift(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
