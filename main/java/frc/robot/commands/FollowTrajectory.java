package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.Constants;


public class FollowTrajectory extends SequentialCommandGroup {
    Trajectory trajectory = new Trajectory();
    //String path = "pathplanner/generatedJSON/TestPath2.wpilib.json";
    private final DriveSubsystem drive;

    public  FollowTrajectory(DriveSubsystem m_drive, String path) {
      drive = m_drive;

    try{
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

    } catch (IOException ex) {

      DriverStation.reportError("Unable to open trajectory:" + path, ex.getStackTrace()); 

    }
      RamseteCommand ramseteCommand =
      new RamseteCommand(
      trajectory,
      drive::getPose,
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
      new SimpleMotorFeedforward(
      Constants.ks,
      Constants.kv,
      Constants.ka),
      Constants.kDriveKinematics,
      drive::getWheelSpeeds,
      new PIDController(Constants.kp, 0, 0),
      new PIDController(Constants.kp, 0, 0),
      drive::tankDriveVolts,
      drive);

      drive.resetOdometry(trajectory.getInitialPose());

      addCommands(

        ramseteCommand.andThen(() -> drive.tankDriveVolts(0,0)));
    

}
 

}

