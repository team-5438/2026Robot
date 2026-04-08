// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Constants;
import frc.robot.Robot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAdjustingShootyWheels extends Command {
  ShootingSubsystem shootingSubsystem;
  InterpolatingAutoAim interpolatingAutoAim;
  SwerveSubsystem swerveSubsystem;
  double defaultSpeed;
  
  /** Creates a new ShootCommand. */
  public AutoAdjustingShootyWheels(ShootingSubsystem shootingSubsystem, SwerveSubsystem swerveSubsystem, InterpolatingAutoAim interpolatingAutoAim, double defaultSpeed) {
    this.shootingSubsystem = shootingSubsystem;
    this.interpolatingAutoAim = interpolatingAutoAim;
    this.defaultSpeed = defaultSpeed;
    this.swerveSubsystem = swerveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shootingSubsystem.shootWheelsRight.set(defaultSpeed);
    shootingSubsystem.shootWheelsLeft.set(-defaultSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(interpolatingAutoAim.interAutoAimOn){
      double distanceX = swerveSubsystem.getPose().getX() - Constants.HUB_X;
      double distanceY =swerveSubsystem.getPose().getY() - Constants.HUB_Y;
      double distanceFromHub = Math.sqrt((Math.pow(distanceX, 2) + Math.pow(distanceY, 2))); //Pythagorean Theorem
  
      if(distanceFromHub >= 3.1){
        shootingSubsystem.shootWheelsRight.set(1);
        shootingSubsystem.shootWheelsLeft.set(-1);
      } else {
        shootingSubsystem.shootWheelsRight.set(0.9);
        shootingSubsystem.shootWheelsLeft.set(-0.9);
      }
    } else {
      shootingSubsystem.shootWheelsRight.set(defaultSpeed);
      shootingSubsystem.shootWheelsLeft.set(-defaultSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shootingSubsystem.shootWheelsRight.set(0);
    shootingSubsystem.shootWheelsLeft.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
