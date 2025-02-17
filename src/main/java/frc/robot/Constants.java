// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.math.geometry.Translation3d;
import swervelib.math.Matter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double DEADBAND = 0.05;

  }
  public static final double maxSpeed = 4.5;



  //elevator
  public static class ElevatorConstants{
    public static final double kElevatorKp = 5;//subject to change. Temporary
    public static final double kElevatorKi = 0;//subject to change. Temporary    
    public static final double kElevatorKd = 0;//subject to change. Temporary

    public static final double kMaxVelocity = Meters.of(4).per(Second).in(MetersPerSecond);
    public static final double kMaxAcceleration = Meters.of(6).per(Second).per(Second).in(MetersPerSecondPerSecond);

    public static final double kElevatorkS = 0.02;
    public static final double kElevatorkG = 0.9;
    public static final double kElevatorkV = 3.8;
    public static final double kElevatorkA = 0.17;


  }
}
