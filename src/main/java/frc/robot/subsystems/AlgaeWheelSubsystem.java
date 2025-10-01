package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class AlgaeWheelSubsystem extends SubsystemBase {
   
    SparkMax climber;
    public AlgaeWheelSubsystem(){
      climber = new SparkMax(46, MotorType.kBrushed);    
    }

    public void AlgaeStart(){
        climber.set(1);
    }

    public void AlgaeStop(){
        climber.set(0);
    }

    public void AlgaeReverse(){
        climber.set(-1);
    }
}