// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HoodAutoAim extends Command {
  ShootingSubsystem shootingSubsystem;
  SwerveSubsystem swerveSubsystem;
  PIDController hoodPID;
  double outputtedAngle;
  CommandPS5Controller operator;
  /** Creates a new HoodAutoAim. */
  public HoodAutoAim(ShootingSubsystem shootingSubsystem, SwerveSubsystem swerveSubsystem, CommandPS5Controller operator) {
    this.shootingSubsystem = shootingSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    this.operator = operator;
    hoodPID = Constants.Shooting.hoodPID;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distanceX = Units.metersToInches(swerveSubsystem.getPose().getX() - Constants.HUB_X);
    double distanceY = Units.metersToInches(swerveSubsystem.getPose().getY() - Constants.HUB_Y);
    double distanceFromHub = Math.sqrt((Math.pow(distanceX, 2) + Math.pow(distanceY, 2))); //Pythagorean Theorem
    /* -------NOTE: PLUG IN DISTANCEFROMHUB TO A FORMULA TO GET CORRECT ANGLE */
    outputtedAngle = 
      -0.000010805*Math.pow(distanceFromHub, 2)
      + 0.00200242*distanceFromHub
      + 0.498052;
    
    shootingSubsystem.hoodAngleMotor.set(hoodPID.calculate(shootingSubsystem.hoodAngleEncoderValue, outputtedAngle));
    System.out.println("distance: " + distanceFromHub + "      output: " + outputtedAngle + "     current: " + shootingSubsystem.hoodAngleEncoderValue);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(operator.povUp().getAsBoolean() || operator.povDown().getAsBoolean()){
      return true;
    }
    return false;
  }
}
