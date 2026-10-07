// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meter;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import yams.mechanisms.swerve.utility.SwerveInputStream;

public class RobotContainer {

  public SwerveDriveSubsystem swerve = new SwerveDriveSubsystem(Filesystem.getDeployDirectory());
  public PS4Controller controller = new PS4Controller(0);

  public boolean headingControlActive = false;

  private final SendableChooser<Command> autoChooser;

  //Array of poses to select from. Pretend to add button to cycle through and see through dashboard. Dope;
  private Pose2d[] poses = {
    new Pose2d(
    new Translation2d
    (Meter.of(4), 
    Meter.of(3)),
    Rotation2d.fromDegrees(0)), 

    new Pose2d(
      new Translation2d(
        Meter.of(3),
        Meter.of(5)),
        Rotation2d.fromDegrees(50)
    )};

    private int currentPose = 0;

  final SwerveInputStream driveStream = swerve.getAngularVelocityStream(
      () -> controller.getLeftX(),
      () -> -controller.getLeftY(),
      () -> controller.getRawAxis(Constants.OperatorConstants.ROTATE_AXIS))
      .withControllerHeadingAxis(
          () -> controller.getRightX(),
          () -> controller.getRightY())
      .withHeadingControl(() -> headingControlActive)
      .withDeadband(0.05)
      .withAllianceRelativeControl();

  public RobotContainer() {

    //pathplanner auto stuff
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("PathPlanning/Selected Auto: ", autoChooser);
    SmartDashboard.putData("PathPlanning/Selected Pose: ",(Sendable) poses[currentPose]);
    configureBindings();
  }

  private void configureBindings() {

    // Ativa e desativa o headingcontrol
    SmartDashboard.putBoolean("HeadingControlActive", headingControlActive);

    // Autoexplicativo
    swerve.setDefaultCommand(swerve.drive(driveStream));

    //Ativa ou desativa o heading control
    new JoystickButton(controller, 2).onTrue(Commands.runOnce(() -> {
      headingControlActive = !headingControlActive;
      SmartDashboard.putBoolean("HeadingControlActive", headingControlActive);
    }).ignoringDisable(true));

    new JoystickButton(controller, 1).onTrue(Commands.runOnce(()->{

    }, swerve)
    );

    // trava as rodas do swerve. Comando pronto
    new JoystickButton(controller, Button.kCross.value).whileTrue(swerve.lockSwerve());

    // reseta o gyro
    new JoystickButton(controller, Button.kTriangle.value).onTrue(swerve.newZero());

    // Setta speeds a partir do elastic para testes
    new JoystickButton(controller, Button.kCross.value).whileTrue(swerve.setSpeedsFromDashboard());
  }

  public Command getAutonomousCommand() {

    return autoChooser.getSelected();
  }
}
