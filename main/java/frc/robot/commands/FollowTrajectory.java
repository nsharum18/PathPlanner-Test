package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.*;
import frc.robot.Constants;


public class FollowTrajectory extends RamseteCommand {

  private final DriveSubsystem drive;
  private final Trajectory trajectory;

  public FollowTrajectory(DriveSubsystem m_drive, Trajectory trajectory) { 
    super(
    trajectory,
    m_drive::getPose,
    new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
    new SimpleMotorFeedforward(
        Constants.ks,
        Constants.kv,
        Constants.ka
    ),
    Constants.kDriveKinematics,
    m_drive::getWheelSpeeds,
    new PIDController(Constants.kp, 0, 0),
    new PIDController(Constants.kp, 0, 0),
    m_drive::tankDriveVolts,
    m_drive);

      this.trajectory = trajectory;
      this.drive = m_drive;
    }

  public void intialize() {

    drive.resetOdometry(trajectory.getInitialPose());
    super.initialize();
  }

  public void execute() {

    super.execute();

  }

  public void end(boolean interrupted) {

    drive.stop();
    super.end(interrupted);


  }


}