// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();

  XboxController DriverSticks = new XboxController(0);

  private final Command TestAuto = new TestAuto(m_drive, m_arm);
  


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_drive.setDefaultCommand(
      
    new RunCommand(
      () -> m_drive.arcadeDrive(-DriverSticks.getLeftY(), DriverSticks.getRightX()), m_drive)
    );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

   /* Trajectory trajectory = new Trajectory();
    String path = "pathplanner/generatedJSON/TestPath2.wpilib.json";

    try{
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

    } catch (IOException ex) {

      DriverStation.reportError("Unable to open trajectory:" + path, ex.getStackTrace()); 

    }
      RamseteCommand ramseteCommand =
      new RamseteCommand(
      trajectory,
      m_drive::getPose,
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
      new SimpleMotorFeedforward(
      Constants.ks,
      Constants.kv,
      Constants.ka),
      Constants.kDriveKinematics,
      m_drive::getWheelSpeeds,
      new PIDController(Constants.kp, 0, 0),
      new PIDController(Constants.kp, 0, 0),
      m_drive::tankDriveVolts,
      m_drive);
      

    m_drive.resetOdometry(trajectory.getInitialPose());
    return ramseteCommand.andThen(() -> m_drive.tankDriveVolts(0,0));*/

  return new TestAuto(m_drive, m_arm);

  }    
}



