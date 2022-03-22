package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;


public class TestAuto extends SequentialCommandGroup {

  public TestAuto(DriveSubsystem m_drive, ArmSubsystem m_arm){

    addCommands(   
    parallel( 
    new FollowTrajectory(m_drive, "pathplanner/generatedJSON/TestPath.wpilib.json"),
    new ArmDown(m_arm)
      ),

    parallel(
    new FollowTrajectory(m_drive, "pathplanner/generatedJSON/TestPath2.wpilib.json"),
    new ArmUp(m_arm)
    )
    );
  }




}