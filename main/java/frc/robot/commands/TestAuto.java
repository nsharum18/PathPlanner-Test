package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;


public class TestAuto extends SequentialCommandGroup {
  String TestPath = "pathplanner/generatedJSON/TestPath.wpilib.json";

  public TestAuto(DriveSubsystem m_drive, ArmSubsystem m_arm, IntakeSubsystem m_intake){

    addCommands(
    parallel(
    new ParallelDeadlineGroup(
      new FollowTrajectory(m_drive, TestPath),

      new Intake(m_intake)
      ),

    sequence(
      new ArmDown(m_arm),

      new WaitCommand(3.5),

      new ArmUp(m_arm))
      ),
    
      new Score(m_intake)
    
    );
  }
}
