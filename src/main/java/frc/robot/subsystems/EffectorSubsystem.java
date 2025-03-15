// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ElevatorConstants;

public class EffectorSubsystem extends SubsystemBase {
    
    private final DigitalInput limitSwitch;
    
    //DigitalInput toplimitSwitch = new DigitalInput(ElevatorConstants.beamBreakDigitalInputID);
    SparkMax effector;

    /** Creates a new Intake. */
    public EffectorSubsystem() {      
        
        // set our effector id
        effector = new SparkMax(43, MotorType.kBrushed);
        
        // add the limit switch
        this.limitSwitch = new DigitalInput(0);

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


    public Command loadUntilBeamBreak() {


        // run until the limit switch returns false
        while( !limitSwitch.get() ) {
            effector.set(0.85);
        }

        // once beam is broken, set speed to 0;
        effector.set(0);

        //shooterSwitch = new DigitalInput(Constants.SHOOTER_SWITCH);
        //toplimitSwitch = new DigitalInput(ElevatorConstants.beamBreakDigitalInputID);
        
        
        
        
        
        return null;

    }


    /*
    public Command getStartShooterCommand() {
        return this.runOnce(() -> {this.setShooterSpeed(0.75);});
    }
    */

    /*
    public Command getShooterCommand() {
        return this.startEnd(() -> {
            this.setShooterSpeed(0.8);
        }, () -> {
            this.setShooterSpeed(0);
        });
    }
    */

    /*
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
    */

    /*
    public Command getRunReverseShooter() {
        return this.startEnd(() -> {
            this.setShooterSpeed(-.10);
        }, () -> {
            this.setShooterSpeed(0);
        });
    }
    */

    /*
    public void setShooterSpeed(double speed){
        effector.set(speed);
    }
    */

} // end class





