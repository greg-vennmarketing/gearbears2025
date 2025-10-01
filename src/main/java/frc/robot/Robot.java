// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.io.IOException;

import com.studica.frc.AHRS;

// Muhammad's Camera Import
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

  UsbCamera camera;

  private AHRS navX; 
  private Command m_autonomousCommand;
 
  public Client cli;
  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    cli = new Client(9480);
    // Shuffleboard.getTab("vision test").add(CommandScheduler.getInstance());

    // camera1 = CameraServer.startAutomaticCapture(0);
    // camera2 = CameraServer.startAutomaticCapture(1);

    m_robotContainer = new RobotContainer(cli);

    
  // m_robotContainer = new RobotContainer();

  //CameraServer.startAutomaticCapture();

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
	m_robotContainer.drivebase.swerveDrive.updateOdometry();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    // Get yaw angle
    SmartDashboard.putNumber("Gyro Yaw", m_robotContainer.drivebase.swerveDrive.getYaw().getDegrees());
    
    // Get angular velocity (rotation rate in degrees/sec)
    SmartDashboard.putNumber("Gyro Rate", m_robotContainer.drivebase.swerveDrive.getFieldVelocity().omegaRadiansPerSecond * 57.3); // convert to degrees
    
    // Get pitch and roll (useful for debugging orientation)
    SmartDashboard.putNumber("Gyro Pitch", m_robotContainer.drivebase.swerveDrive.getPitch().getDegrees());
    SmartDashboard.putNumber("Gyro Roll", m_robotContainer.drivebase.swerveDrive.getRoll().getDegrees());
    
    // Get full pose (includes gyro heading)
    Pose2d pose = m_robotContainer.drivebase.swerveDrive.getPose();
    SmartDashboard.putNumber("Pose X", pose.getX());
    SmartDashboard.putNumber("Pose Y", pose.getY());
    SmartDashboard.putNumber("Pose Rotation", pose.getRotation().getDegrees());

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
	m_robotContainer.drivebase.swerveDrive.drive(new Translation2d(0, 0), 0, false, false);
  }


  /* 
    @Override
    public void robotInit() 
    {
      navX = new


    }*/


  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
        
    // Get yaw angle
    SmartDashboard.putNumber("Gyro Yaw", m_robotContainer.drivebase.swerveDrive.getYaw().getDegrees());
    
    // Get angular velocity (rotation rate in degrees/sec)
    SmartDashboard.putNumber("Gyro Rate", m_robotContainer.drivebase.swerveDrive.getFieldVelocity().omegaRadiansPerSecond * 57.3); // convert to degrees
    
    // Get pitch and roll (useful for debugging orientation)
    SmartDashboard.putNumber("Gyro Pitch", m_robotContainer.drivebase.swerveDrive.getPitch().getDegrees());
    SmartDashboard.putNumber("Gyro Roll", m_robotContainer.drivebase.swerveDrive.getRoll().getDegrees());
    
    // Get full pose (includes gyro heading)
    Pose2d pose = m_robotContainer.drivebase.swerveDrive.getPose();
    SmartDashboard.putNumber("Pose X", pose.getX());
    SmartDashboard.putNumber("Pose Y", pose.getY());
    SmartDashboard.putNumber("Pose Rotation", pose.getRotation().getDegrees());

    try {
      if (cli.counter % 2 == 0) {
        cli.askForIn();
      } else {
        cli.read();
      }
    } catch (IOException e) {
     System.out.println("vision network failed");
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}


  // Muhammad's code for camera feed on shuffleboard
//  @Override
//  public void robotInit() {
//    CameraServer.startAutomaticCapture();
//
//    ShuffleboardTab tab = Shuffleboard.getTab("Camera Feed");
//    tab.add("Limelight Stream", CameraServer.getServer());
//  }
}
