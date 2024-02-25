// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.io.PilotingControls;
import frc.robot.subsystems.Swerve.SwerveDriveTrain;
import frc.robot.util.SteelTalonsLogger;

public class RobotContainer {

  private SwerveDriveTrain drivetrain;

  public RobotContainer() {

    drivetrain = new SwerveDriveTrain();
    drivetrain.setDefaultCommand(drivetrain.getDriveCommand(new CommandXboxController(0)));
    new SteelTalonsLogger();


    new PilotingControls(new CommandXboxController(0));
  }
}