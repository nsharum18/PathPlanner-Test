// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;


public class IntakeSubsystem extends SubsystemBase {

  //creates intake motor
  private final WPI_TalonFX intake = new WPI_TalonFX(7);

  /** Creates a new ArmSubsytem. */
  public IntakeSubsystem() {
  
        //sets brake
        intake.setNeutralMode(NeutralMode.Coast);
    
        //sets rotation
        intake.setInverted(TalonFXInvertType.Clockwise);
  }

  public void intake(double speed) {

    intake.set(speed * .6);
  }

  public void score(double speed) {

    intake.set(speed * 1);
  }

  public void stop() {

    intake.set(0);

  }

  public double getIntakeEnc() {

    return intake.getSelectedSensorPosition() / 2048;

  }

  public void resetIntakeEnc() {

    intake.setSelectedSensorPosition(0);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}