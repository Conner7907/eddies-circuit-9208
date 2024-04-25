// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;


public class LimelightSubsystem extends SubsystemBase {
  public boolean m_LimelightHasValidTarget;
  public double m_Limelight_tx;
  public double m_Limelight_ty;
  //public double m_Limelight_tz;
  //public double m_Limelight_ty;
  public double m_Limelight_ta;    


  public final Solenoid PurpleLight = new Solenoid(13, PneumaticsModuleType.CTREPCM, 6);

 
  
 public LimelightSubsystem() {


  }
  
  public void setPurpleLightsOn(){
    System.out.println("Purple Light ON");
    PurpleLight.set(true);
  }

  public void setPurpleLightsOff(){
    PurpleLight.set(false);
  }


  @Override
  public void periodic() {
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
      // These numbers must be tuned for your Robot!  Be careful!
      /* 
      
      final double STEER_K = 0.03;                    // how hard to turn toward the target
      final double DRIVE_K = 0.26;                    // how hard to drive fwd toward the target
      final double DESIRED_TARGET_AREA = 13.0;        // Area of the target when the robot reaches the wall
      final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast
      */

      if (tv < 1.0)
      {
        m_LimelightHasValidTarget = false;
        //m_LimelightDriveCommand = 0.0;
        //m_LimelightSteerCommand = 0.0;
        //return;
      }else

      m_LimelightHasValidTarget = true;

      /* 
      // Start with proportional steering
      double steer_cmd = tx * STEER_K;
      m_LimelightSteerCommand = steer_cmd;

      // try to drive forward until the target area reaches our desired area
      double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

      // don't let the robot drive too fast into the goal
      if (drive_cmd > MAX_DRIVE)
      {
        drive_cmd = MAX_DRIVE;
      }
      m_LimelightDriveCommand = drive_cmd;*/
      //ta1 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
      SmartDashboard.putNumber("tv", tv);
      SmartDashboard.putNumber("tx", tx);
      SmartDashboard.putNumber("ty", ty);
      SmartDashboard.putNumber("ta", ta);
      SmartDashboard.putBoolean("Limelight Target", m_LimelightHasValidTarget);
      //System.out.println(m_LimelightHasValidTarget);
      m_Limelight_tx = tx;
      m_Limelight_ty = ty;
      m_Limelight_ta = ta;


  }
    public boolean getTv(){
    //System.out.println(m_LimelightHasValidTarget + " LimelightSubsytem");
    return m_LimelightHasValidTarget;
  }

    public double getTx(){
    return m_Limelight_tx;
  }

      public double getTy(){
    return m_Limelight_ty;
  }

      public double getTa(){
    return m_Limelight_ta;
  }
}
