// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.math.SwerveMath;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpencerAutoAim extends Command {
  SwerveSubsystem swerveSubsystem;
  DoubleSupplier PIDOutputDoubleSupplier;
  PIDController anglePID;
  Pose2d robotPose2d;
  double robotX, robotY, 
         diffX, diffY,
         desiredAngle,
         PIDOutput;
  public boolean autoAimOn;
  CommandXboxController driver;
  Command autoaimcommand;
  DoubleSupplier rightRotation, translationX, translationY;
  ShootingSubsystem shootingSubsystem;
  /** Creates a new SpencerAutoAim. */
  public SpencerAutoAim(CommandXboxController driver, SwerveSubsystem swerveSubsystem, ShootingSubsystem shootingSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    this.shootingSubsystem = shootingSubsystem;

    anglePID = Constants.anglePID;
    this.driver = driver;
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    autoAimOn = true;
      translationX = () -> -MathUtil.applyDeadband(driver.getLeftY(), Constants.Driver.leftStick.Y);
      translationY = () -> -MathUtil.applyDeadband(driver.getLeftX(), Constants.Driver.leftStick.X);
      rightRotation = () -> MathUtil.applyDeadband(driver.getRightX(), Constants.Driver.rightStick.X);
      PIDOutputDoubleSupplier = () -> {
      robotPose2d = swerveSubsystem.getPose();
      robotX = robotPose2d.getMeasureX().in(Meters);
      robotY = robotPose2d.getMeasureY().in(Meters);
      diffX = Constants.HUB_X - robotX;
      diffY = Constants.HUB_Y - robotY;
      desiredAngle = (diffX == 0) ? 0 : Math.atan2(diffY, diffX);
      PIDOutput = anglePID.calculate(robotPose2d.getRotation().getRadians(), desiredAngle);
      return PIDOutput*0.2;
    };

    shootingSubsystem.shootWheels.set(0.1);

    // autoaimcommand = swerveSubsystem.driveCommand(translationX, translationY, PIDOutputDoubleSupplier);
    // autoaimcommand.schedule();
  }

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("blehblehblbehlbhelbeh");
          swerveSubsystem.swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                            translationX.getAsDouble() * swerveSubsystem.swerveDrive.getMaximumChassisVelocity(),
                            translationY.getAsDouble() * swerveSubsystem.swerveDrive.getMaximumChassisVelocity()), 0.8),
                        Math.pow(PIDOutputDoubleSupplier.getAsDouble(), 3) * swerveSubsystem.swerveDrive.getMaximumChassisAngularVelocity(),
                        true,
                        false);


    /*
     * get robot pose
     * get robot x and y
     * subtract robot x from hoop x
     * subtract robot y from hoop y
     * arctan(diff) = desired angle
     * (edge case: if diffX = 0, then we know we must point at 0 theta)
     * PID get to desired angle
     * 
     */

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    autoAimOn = false;
    shootingSubsystem.shootWheels.set(0);
    // if(autoaimcommand != null){
    //   autoaimcommand.cancel();
    // }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(rightRotation.getAsDouble() != 0){
      System.out.println("JASLKDFJLSAJFLKDSAMFLKDSAJFLKDSAJFLKSAJFLKSAJF");
      return true;
    }
    System.out.println("ch");
    return false;
  }
}
