// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EffectorSubsystem extends SubsystemBase {
  SparkMax effector;
  /** Creates a new Intake. */
  public EffectorSubsystem() {
    effector = new SparkMax(43, MotorType.kBrushed);
    

  }
  public void setShooterSpeed(double speed){
    effector.set(speed);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }


  public Command getStartShooterCommand() {
    return this.runOnce(() -> {this.setShooterSpeed(0.75);});
  }
  public Command getShooterCommand() {
    return this.startEnd(() -> {
      this.setShooterSpeed(1);
    }, () -> {
      this.setShooterSpeed(0);
    });
  }

  public Command getStopCommand() {
    return this.runOnce(() -> {this.setShooterSpeed(0);});
  }
   public Command getSlowShootCommand() {
    return this.startEnd(() -> {
      this.setShooterSpeed(.08);
    }, () -> {
      this.setShooterSpeed(0);
    });
  }

  public Command getRunReverseShooter() {
    return this.startEnd(() -> {
      this.setShooterSpeed(-.10);
    }, () -> {
      this.setShooterSpeed(0);
    });
  }

  public Command shoot(){

    effector.set(0.85);
        return null;

  }
 

  public Command stop(){
    effector.set(0);

    return null;
  }


public Command intake(){

  effector.set(0.3);
      return null;

}
}




