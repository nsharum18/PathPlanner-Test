// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

public class ArmSubsystem extends SubsystemBase {

  //creates arm motor
  private final WPI_TalonFX arm = new WPI_TalonFX(5);


  /** Creates a new ArmSubsytem. */
  public ArmSubsystem() {

    //sets brake
    arm.setNeutralMode(NeutralMode.Brake);
    
    //sets rotation
    arm.setInverted(TalonFXInvertType.CounterClockwise);

  }

  public void resetArmEnc() {

    //sets arm encoder to 0
    arm.setSelectedSensorPosition(0);
  }

  public void armSetStartEnc() {

    //sets arm encoder for start position
    arm.setSelectedSensorPosition(0);
  }

  public double getArmEnc() {

  return arm.getSelectedSensorPosition() / 2048;

  }

  public void armSetIntake() {

    //sets arm for intake
    arm.set(-1);
    
  }

  public void armSetScore() {

    //sets arm for Score
    arm.set(1);
    
  }

  public void armSetStart() {

    //sets arm for Start
    arm.set(.25);
    
  }

  public void armManualDown() {

    //manual down
    arm.set(-.25);
  }

  public void armSetClimb() {

    //sets arm for climbing
    
  }

  public void armStop() {

    //stops arm
    arm.set(0);
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