// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ElevatorSubsytem extends SubsystemBase {

    private final DCMotor m_elevatorGearbox = DCMotor.getNEO(1);
    private final SparkMax m_motor = new SparkMax(41, SparkLowLevel.MotorType.kBrushless);
    private final SparkMaxSim m_motorSIm = new SparkMaxSim(m_motor, m_elevatorGearbox);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final ProfiledPIDController m_controller = new ProfiledPIDController(ElevatorConstants.kElevatorKp, 
                                                                                    ElevatorConstants.kElevatorKi, 
                                                                                    ElevatorConstants.kElevatorKd, 
                                                                                    new Constraints(ElevatorConstants.kMaxVelocity, 
                                                                                    ElevatorConstants.kMaxAcceleration) );
    private final ElevatorFeedforward m_feedForward = new ElevatorFeedforward(ElevatorConstants.kElevatorkS, 
                                                                                ElevatorConstants.kElevatorkG, 
                                                                                ElevatorConstants.kElevatorkV, 
                                                                                ElevatorConstants.kElevatorkA);
    private final ElevatorSim m_elevatorSim = null;




}
