package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ClimberSubsystem extends SubsystemBase {
   
SparkMax climber;
    public ClimberSubsystem(){

      climber = new SparkMax(45, MotorType.kBrushed);    
    }




    public void ClimbStart(){
        climber.set(1);
    }
     public void ClimbStop(){
        climber.set(0);
  
    }


    public void ClimbReverse(){
        climber.set(-1);
    }






     public Command getClimbStopCommand() {
        return this.runOnce(() -> {this.ClimbStop();});
      }
      public Command getRunClimb(){
        return this.startEnd(()->ClimbStart(), ()->ClimbStop());
    }


          public Command getRunClimbReverse(){
        return this.startEnd(()->ClimbReverse(), ()->ClimbStop());
    }




    }




