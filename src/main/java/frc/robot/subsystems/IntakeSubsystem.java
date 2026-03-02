// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  public TalonFX intakeDeploy; //part that extends the intake out of the frame
  public TalonFX intakeSpinny; //wheels to intake balls
  public DutyCycleEncoder deployEncoder;
  public double deployEncoderDistance;
  public PIDController intakePID;

  /* Shuffleboard stuff */
  public ShuffleboardTab tab;
  public GenericEntry deployEncoderEntry;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeDeploy = new TalonFX(Constants.Intake.intakeDeployID);
    intakeSpinny = new TalonFX(Constants.Intake.intakeSpinnyID);
    deployEncoder = new DutyCycleEncoder(Constants.Intake.deployEncoderID);

    tab = Shuffleboard.getTab("Intake Subsystem");
    deployEncoderEntry = tab.add("deployEncoder", 0.0).getEntry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    deployEncoderDistance = deployEncoder.get();
    deployEncoderEntry.setDouble(deployEncoderDistance);

  }
}
