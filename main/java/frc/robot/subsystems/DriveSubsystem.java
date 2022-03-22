// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  private final WPI_TalonFX LMaster = new WPI_TalonFX(Constants.LEFT_DRIVE_MASTER);
  private final WPI_TalonFX LFollower = new WPI_TalonFX(Constants.LEFT_DRIVE_FOLLOWER);
  private final WPI_TalonFX RMaster = new WPI_TalonFX(Constants.RIGHT_DRIVE_MASTER);
  private final WPI_TalonFX RFollower = new WPI_TalonFX(Constants.RIGHT_DRIVE_FOLLOWER);

  private final DifferentialDrive m_drive = new DifferentialDrive(LMaster, RMaster);
  
  private final AHRS m_navx = new AHRS(edu.wpi.first.wpilibj.SPI.Port.kMXP);

  private final DifferentialDriveOdometry m_odometry;

  Pose2d pose;

  NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
  NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");

  private final Field2d m_field = new Field2d();

  


  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {

    LFollower.follow(LMaster);
    LFollower.setInverted(InvertType.FollowMaster);
    RFollower.follow(RMaster);
    RFollower.setInverted(InvertType.FollowMaster);

    LMaster.setNeutralMode(NeutralMode.Brake);
    LFollower.setNeutralMode(NeutralMode.Brake);
    RMaster.setNeutralMode(NeutralMode.Brake);
    RFollower.setNeutralMode(NeutralMode.Brake);

    LMaster.setInverted(TalonFXInvertType.Clockwise);
    RMaster.setInverted(TalonFXInvertType.CounterClockwise);

    LMaster.configOpenloopRamp(Constants.RAMP_TIME);
    RMaster.configOpenloopRamp(Constants.RAMP_TIME);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_navx.getRotation2d());

    SmartDashboard.putData("Field", m_field);

 

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  m_odometry.update(m_navx.getRotation2d(), 
    nativeUnitsToDistanceMeters(getLMasterEncoder()), 
    nativeUnitsToDistanceMeters(getRMasterEncoder()));

    SmartDashboard.putNumber("LEncoder", getLMasterEncoder());
    SmartDashboard.putNumber("REncoder", getRMasterEncoder());
    SmartDashboard.putNumber("Heading", getHeading());

    var translation = m_odometry.getPoseMeters().getTranslation();
    m_xEntry.setNumber(translation.getX());
    m_yEntry.setNumber(translation.getY());

    m_field.setRobotPose(m_odometry.getPoseMeters());

    Field2d m_field = new Field2d();
    SmartDashboard.putData(m_field);


  }


  public Pose2d getPose() {

    return m_odometry.getPoseMeters();

  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    double left_meters_sec = 10 * nativeUnitsToDistanceMeters(getLMasterRate());
    double right_meters_sec = 10 * nativeUnitsToDistanceMeters(getRMasterRate());

    return new DifferentialDriveWheelSpeeds(left_meters_sec, right_meters_sec);

  }

  public void resetOdometry(Pose2d pose) {

    resetEncoders();
    m_odometry.resetPosition(pose, m_navx.getRotation2d());

  }

  public void arcadeDrive(double fwd, double rot) {

    m_drive.arcadeDrive(fwd * Constants.DRIVE_SPEED, rot * Constants.TURN_SPEED);

  }

  public void stop() {

    m_drive.arcadeDrive(0, 0);

  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    LMaster.setVoltage(leftVolts);
    RMaster.setVoltage(rightVolts);
    SmartDashboard.putNumber("LeftVolts",leftVolts);
    SmartDashboard.putNumber("RightVolts", rightVolts);
    m_drive.feed();
  }

  public void resetEncoders() {

    LMaster.setSelectedSensorPosition(0, 0, 0);
    RMaster.setSelectedSensorPosition(0, 0, 0);

  }

  public double getLMasterEncoder() {

    return LMaster.getSelectedSensorPosition();
    
  }

  public double getRMasterEncoder() {

    return RMaster.getSelectedSensorPosition();

  }

  public double getLMasterRate() {

    return LMaster.getSelectedSensorVelocity();

  }

  public double getRMasterRate() {

    return RMaster.getSelectedSensorVelocity();

  }

  public void zeroHeading() {

    m_navx.reset();

  }

  public double getHeading() {

    return Math.IEEEremainder(m_navx.getAngle(), 360) * (Constants.kGyroReversed ? -1.0 : 1.0);

  }

  public double getTurnRate() {

    return -m_navx.getRate();
    
  }


  private double nativeUnitsToDistanceMeters(double sensorCounts){
    double motorRotations = (double)sensorCounts / Constants.EncoderCPR;
    double wheelRotations = motorRotations / Constants.gearRatio;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(Constants.wheelradius));
    return positionMeters;


  }
  
  
    


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
