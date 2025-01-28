// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  SparkMax shooterLeft;
  SparkMax shooterRight;
  /** Creates a new Intake. */
  public Shooter() {
    shooterRight = new SparkMax(42, SparkMax.MotorType.kBrushless);
    shooterLeft = new SparkMax(41, SparkMax.MotorType.kBrushless);
  }
  public void setShooterSpeed(double speed){
    shooterLeft.set(speed);
    shooterRight.set(-speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }


  public Command getStartShooterCommand() {
    return this.runOnce(() -> {this.setShooterSpeed(0.35);});
  }
  public Command getShooterCommand() {
    return this.startEnd(() -> {
      this.setShooterSpeed(0.35);
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
}





