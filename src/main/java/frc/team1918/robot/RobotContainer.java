// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1918.robot;

import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandStadiaController;
import frc.team1918.robot.constants.*;
import frc.team1918.robot.subsystems.SwerveSubsystem;
import frc.team1918.robot.utils.InputAxis;

public class RobotContainer {
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
  private final CommandStadiaController dj = new CommandStadiaController(OIConstants.JoyDriverID);

  public RobotContainer() {
    final InputAxis m_fieldX = new InputAxis("Forward", dj::getLeftY)
      .withDeadband(OIConstants.kMinDeadband)
      .withInvert(true)
      .withSquaring(true);
    final InputAxis m_fieldY = new InputAxis("Strafe", dj::getLeftX)
      .withDeadband(OIConstants.kMinDeadband)
      .withInvert(true)
      .withSquaring(true);
    final InputAxis m_rotate = new InputAxis("Rotate", dj::getRightX)
      .withDeadband(OIConstants.kMinDeadband)
      .withInvert(true);
    final InputAxis m_angleX = new InputAxis("Angle X", dj::getRightX);
    final InputAxis m_angleY = new InputAxis("Angle Y", dj::getRightY);

    configureBindings();

    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(m_fieldX, m_fieldY, m_angleX, m_angleY);
    Command driveFieldOrientedAngularVelocity = drivebase.driveCommand(m_fieldX, m_fieldY, m_rotate);
    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(m_fieldX, m_fieldY, m_rotate);
    drivebase.setDefaultCommand(
      !RobotBase.isSimulation() 
        ? driveFieldOrientedAngularVelocity //driveFieldOrientedDirectAngle or driveFieldOrientedAngularVelocity
        : driveFieldOrientedDirectAngleSim
    );

  }

  private void configureBindings() {
    dj.frame().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    dj.stadia().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));

    //hold B to drive to a given pose
    dj.b().whileTrue(
      Commands.deferredProxy(() -> drivebase.driveToPose(
        new Pose2d(new Translation2d(4,4), Rotation2d.fromDegrees(0)))
      )
    );
  }

  public Command getAutonomousCommand() {
    // return Commands.print("No autonomous command configured");
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setDriveMode() {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
