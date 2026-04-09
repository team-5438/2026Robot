// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HungryHippo extends Command {
  /** Creates a new HungryHippo. */
  IntakeSubsystem intakeSubsystem;
  PIDController intakePID;

  /**
   * 
   * Runs the intake up and down a lot...Good for trying to shoot and stuff...Great name
   * 
   */
  public HungryHippo(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    intakePID = Constants.Intake.intakeDeployPID;

    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intakeSubsystem.deployEncoderDistance < 0.18){
      intakeSubsystem.intakeDeployLeft.set(intakePID.calculate(intakeSubsystem.deployEncoderDistance, 0.275));
      intakeSubsystem.intakeDeployRight.set(intakePID.calculate(intakeSubsystem.deployEncoderDistance, 0.275));
    } else {
      intakeSubsystem.intakeDeployLeft.set(intakePID.calculate(intakeSubsystem.deployEncoderDistance, 0.105));
      intakeSubsystem.intakeDeployRight.set(intakePID.calculate(intakeSubsystem.deployEncoderDistance, 0.105));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
