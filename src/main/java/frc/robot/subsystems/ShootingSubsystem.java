// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShootingSubsystem extends SubsystemBase {
  public TalonFX shootWheelsLeft;
  public TalonFX shootWheelsRight;
  public TalonFX feedWheels;
  public SparkMax hoodAngleMotor;
  public DutyCycleEncoder hoodAngleEncoder;
  public double hoodAngleEncoderValue;

  /* Shuffleboard stuff */
  public ShuffleboardTab tab;
  public GenericEntry hoodAngleEncoderEntry;

  /** Creates a new ShootingSubsystem. */
  public ShootingSubsystem() {
    shootWheelsLeft = new TalonFX(Constants.Shooting.shootWheelsLeftID);
    shootWheelsRight = new TalonFX(Constants.Shooting.shootWheelsRightID);
    feedWheels = new TalonFX(Constants.Shooting.feedWheelsID);
    hoodAngleMotor = new SparkMax(Constants.Shooting.hoodAngleMotorID, MotorType.kBrushless);
    hoodAngleEncoder = new DutyCycleEncoder(Constants.Shooting.hoodAngleEncoderID);

    tab = Shuffleboard.getTab("Shooting Subsystem");
    hoodAngleEncoderEntry = tab.add("hoodEncoder", 0.0).getEntry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    hoodAngleEncoderValue = hoodAngleEncoder.get();
    hoodAngleEncoderEntry.setDouble(hoodAngleEncoderValue);
  }
}
