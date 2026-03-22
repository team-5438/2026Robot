// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.Chooser;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {
  public QuestNav questNav;
  SwerveSubsystem swerveSubsystem;
  SendableChooser<Chooser> initialChooser;
  Pose2d initialPose2d;
  Pose3d initialPose3d, questPose;


  /** Creates a new QuestNavSubsystem. */
  public QuestNavSubsystem(SwerveSubsystem swerveSubsystem, Vision vision, SendableChooser<Chooser> initialChooser) {
    this.swerveSubsystem = swerveSubsystem;
    questNav = new QuestNav();

    this.initialChooser = initialChooser;
    initialPose2d = initialChooser.getSelected().getInitialPose(vision);
    initialPose3d = new Pose3d(initialPose2d);
    questPose = initialPose3d.transformBy(Constants.QuestNav.ROBOT_TO_QUEST);
    questNav.setPose(questPose);
    swerveSubsystem.resetOdometry(initialPose2d);

    initialChooser.onChange((chooser) -> {
      initialPose2d = initialChooser.getSelected().getInitialPose(vision);
      initialPose3d = new Pose3d(initialPose2d);
      questPose = initialPose3d.transformBy(Constants.QuestNav.ROBOT_TO_QUEST);
      questNav.setPose(questPose);
      swerveSubsystem.resetOdometry(initialPose2d);
    });
    
  }

  @Override
  public void periodic() {
    questNav.commandPeriodic();

    // Get the latest pose data frames from the Quest
    PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

    // Loop over the pose data frames and send them to the pose estimator
    for (PoseFrame questFrame : questFrames) {
      // Make sure the Quest was tracking the pose for this frame
      if (questFrame.isTracking()) {
        // Get the pose of the Quest
        Pose3d questPose = questFrame.questPose3d();
        // Get timestamp for when the data was sent
        double timestamp = questFrame.dataTimestamp();

        // Transform by the mount pose to get your robot pose
        Pose3d robotPose = questPose.transformBy(Constants.QuestNav.ROBOT_TO_QUEST.inverse());

        // You can put some sort of filtering here if you would like!
        if((robotPose.getMeasureY().in(Inches) <= 317.69 || robotPose.getMeasureX().in(Inches) <= 651.22) && (robotPose.getX() > 0 && robotPose.getY() > 0)){
          // Add the measurement to our estimator
          swerveSubsystem.swerveDrive.addVisionMeasurement(robotPose.toPose2d(), timestamp, Constants.QuestNav.QUESTNAV_STD_DEVS);
        }
      }
    }
  }
}
