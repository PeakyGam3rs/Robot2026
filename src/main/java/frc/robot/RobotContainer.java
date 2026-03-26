// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.commands.CommandShoot;
import frc.robot.commands.CommandStopShoot;
import frc.robot.subsystems.BallFondlerSubsystem;
import frc.robot.subsystems.WheeeeelSubsystem;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.CommandIntake;
import frc.robot.commands.CommandOrbitGoal;
import frc.robot.commands.CommandReverseIntake;

public class RobotContainer {
  public final BallFondlerSubsystem ballFondlerSubsystem = new BallFondlerSubsystem();
  public final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();

  public final CommandXboxController m_driverController = new CommandXboxController(
      OIConstants.kDriverControllerPort);
  public final CommandXboxController m_shooterController = new CommandXboxController(
      OIConstants.kShootControllerPort);

  public WheeeeelSubsystem m_robotDrive;

  public RobotContainer() {
    m_robotDrive = new WheeeeelSubsystem();

    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

    configureBindings();

    NamedCommands.registerCommand("shoot", new CommandShoot(ballFondlerSubsystem));
    NamedCommands.registerCommand("stopShoot", new CommandStopShoot(ballFondlerSubsystem));
    NamedCommands.registerCommand("intake", new CommandIntake(ballFondlerSubsystem));
  }

  private void configureBindings() {
    m_shooterController.rightBumper()
        .whileTrue(new CommandShoot(ballFondlerSubsystem));

    m_shooterController.leftBumper()
        .whileTrue(new CommandIntake(ballFondlerSubsystem));

    m_shooterController.a()
        .whileTrue(new CommandReverseIntake(ballFondlerSubsystem));

    // Hold RB on driver controller: snap to orbit radius around goal, left X strafe along arc
    m_driverController.rightBumper()
        .whileTrue(new CommandOrbitGoal(
            m_robotDrive,
            () -> MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband)));
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("AutoFondler");
  }
}
