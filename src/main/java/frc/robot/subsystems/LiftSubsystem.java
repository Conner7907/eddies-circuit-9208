// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LiftSubsystem extends SubsystemBase {

  public CANSparkMax ClimbingMotorRight = new CANSparkMax(16, MotorType.kBrushless);
  public CANSparkMax ClimbingMotorLeft = new CANSparkMax(17, MotorType.kBrushless);

  public RelativeEncoder ClimbRightEncoder = ClimbingMotorRight.getEncoder();
  public RelativeEncoder ClimbLeftEncoder = ClimbingMotorLeft.getEncoder();

  public final DigitalInput RightLimitSwitch = new DigitalInput(1);  
  public final DigitalInput LeftLimitSwitch = new DigitalInput(2);     

  /** Creates a new LiftSubsytem. */
  public LiftSubsystem() {
    ClimbingMotorRight.setInverted(false);
    ClimbingMotorLeft.setInverted(true);
    ClimbingMotorLeft.setIdleMode(IdleMode.kBrake);
    ClimbingMotorRight.setIdleMode(IdleMode.kBrake);
    ClimbLeftEncoder.setPosition(0);
    ClimbRightEncoder.setPosition(0);
  }

  public void resetClimbEncoder(){
    ClimbLeftEncoder.setPosition(0);
    ClimbRightEncoder.setPosition(0);
  }

  public void setRightLift(double rightArmSpeed){
    ClimbingMotorRight.set(rightArmSpeed);
  }

  public void setLeftLift(double leftArmSpeed){
    ClimbingMotorLeft.set(leftArmSpeed);
  }

  public double getRightClimbEncoder(){
    return ClimbRightEncoder.getPosition();
  }

  public double getLeftClimbEncoder(){
    return ClimbLeftEncoder.getPosition();
  }

  public boolean getRightLimit(){
    return RightLimitSwitch.get();
  }

  public boolean getLeftLimit(){
    return LeftLimitSwitch.get();

  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Right Encoder", ClimbRightEncoder.getPosition());
    SmartDashboard.putNumber("Left Encoder", ClimbLeftEncoder.getPosition());
    SmartDashboard.putBoolean("Right Limit Switch", RightLimitSwitch.get());
    SmartDashboard.putBoolean("Left Limit Switch", LeftLimitSwitch.get());
  }
}
