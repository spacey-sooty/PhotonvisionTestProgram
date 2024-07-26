// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.util.datalog.ProtobufLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  PhotonCamera m_camera = new PhotonCamera("photonvision");
  AprilTagFieldLayout m_fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
  StructLogEntry<Pose3d> m_poseEntry = StructLogEntry.create(DataLogManager.getLog(), "PhotonVision/Pose",
      Pose3d.struct);
  ProtobufLogEntry<PhotonPipelineResult> m_resultEntry = ProtobufLogEntry.create(DataLogManager.getLog(),
      "PhotonVision/Result", PhotonPipelineResult.proto);

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    DataLogManager.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    var result = m_camera.getLatestResult();
    var target = result.getBestTarget();
    Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
        m_fieldLayout.getTagPose(target.getFiducialId()).get(), new Transform3d());
    m_poseEntry.append(robotPose);
    m_resultEntry.append(result);
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
