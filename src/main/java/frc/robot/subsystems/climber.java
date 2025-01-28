package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class climber extends SubsystemBase {
    
SparkMax ClimberMotor;
    public climber(){
        ClimberMotor = new SparkMax(46, SparkMax.MotorType.kBrushed);
    }


    public void Climb(){
        ClimberMotor.set(1);
    }
     public void ClimbStop(){
        ClimberMotor.set(0);
    }

    public void ClimbReverse(){
        ClimberMotor.set(0);
    }



     public Command getClimbStopCommand() {
        return this.runOnce(() -> {this.ClimbStop();});
      }
      public Command getRunClimb(){
        return this.startEnd(()->Climb(), ()->ClimbStop());
    }

          public Command getRunClimbReverse(){
        return this.startEnd(()->ClimbReverse(), ()->ClimbStop());
    }


    }


