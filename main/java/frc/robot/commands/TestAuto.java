package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;


public class TestAuto extends SequentialCommandGroup {

  public TestAuto(DriveSubsystem m_drive){

    addCommands(
      
    new FollowTrajectory(m_drive, PathPlanner.loadPath("TestPath", 1, 1))

   // new FollowTrajectory(m_drive, PathPlanner.loadPath("TestPath2", 1, 1))

    );

  }




}