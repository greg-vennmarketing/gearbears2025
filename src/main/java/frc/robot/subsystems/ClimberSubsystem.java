// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  SparkMax effector;
  /** Creates a new Intake. */
  public ClimberSubsystem() {
    effector = new SparkMax(45, MotorType.kBrushed);

  }
  public void setShooterSpeed(double speed){
    effector.set(speed);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

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




