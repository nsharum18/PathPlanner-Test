// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ArmUp extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
    
    private final ArmSubsystem m_arm;

    public ArmUp(ArmSubsystem subsystem) {

      m_arm = subsystem;
      addRequirements(m_arm);

    }
    
    // Use addRequirements() here to declare subsystem dependencies.


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_arm.armSetScore();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_arm.armSetScore();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_arm.armStop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_arm.getArmEnc() >= 0;
  }
}