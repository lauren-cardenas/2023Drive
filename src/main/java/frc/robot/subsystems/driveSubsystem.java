// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class driveSubsystem extends SubsystemBase {
  /** Creates a new driveSubsystem. */
  private final WPI_TalonFX m_frontLeft = new WPI_TalonFX(DriveConstants.kLeftDrivePort1);
  private final WPI_TalonFX m_backLeft = new WPI_TalonFX(DriveConstants.kLeftDrivePort2);
  private final WPI_TalonFX m_frontRight = new WPI_TalonFX(DriveConstants.kRightDrivePort1);
  private final WPI_TalonFX m_backRight = new WPI_TalonFX(DriveConstants.kRightDrivePort2);

  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_frontLeft, m_backLeft);
  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_frontRight, m_backRight);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  
  public driveSubsystem() {
    m_rightMotors.setInverted(true);

    m_drive.setSafetyEnabled(false);

    m_frontLeft.configFactoryDefault();
    m_frontRight.configFactoryDefault();
    m_backLeft.configFactoryDefault();
    m_backRight.configFactoryDefault();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
    displayEncoderValues();
  }

  public void tankDrive(double left, double right){
    m_drive.tankDrive(left, right);
  }

  public void displayEncoderValues(){
    SmartDashboard.putNumber("Right Data", getRightWheelPosition());
    SmartDashboard.putNumber("Left Data", getLeftWheelPosition());
  }

  private double getLeftWheelPosition(){
    return (m_frontLeft.getSelectedSensorPosition() * DriveConstants.kWheelDiameterMeters * Math.PI
     / DriveConstants.kEncoderCPR) / DriveConstants.kGearRatio;
  }
  
  private double getRightWheelPosition(){
    return (-m_frontRight.getSelectedSensorPosition() * DriveConstants.kWheelDiameterMeters * Math.PI
     / DriveConstants.kEncoderCPR) / DriveConstants.kGearRatio;
  }

  public void resetEncoders(){
    m_frontLeft.setSelectedSensorPosition(0);
    m_frontRight.setSelectedSensorPosition(0);
  }
}
