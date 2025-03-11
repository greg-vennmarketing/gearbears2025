// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

/** An example command that uses an example subsystem. */
public class L1Command extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubsystem m_elevator;

  /**
   * Creates a new ExampleCommand.
   *
   * @param ElevatorSubsystem The subsystem used by this command.
   */
  public L1Command(ElevatorSubsystem subsystem) {
    m_elevator = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setPositionInches(ElevatorConstants.L1);
    System.out.println("L1Command started.");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("L1Command ended.");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_elevator.getHeightInches() - ElevatorConstants.L1) < 0.5;
  }
}
