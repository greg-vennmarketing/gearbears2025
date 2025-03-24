package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class AlgaeSubsystem extends SubsystemBase {
   
SparkMax climber;
    public AlgaeSubsystem(){

      climber = new SparkMax(44, MotorType.kBrushed);    
    }




    public void AlgaeStart(){
        climber.set(0.8);
    }
     public void AlgaeStop(){
        climber.set(0);
  
    }


    public void AlgaeReverse(){
        climber.set(-0.7);
    }







    }




