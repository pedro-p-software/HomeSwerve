// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import yams.mechanisms.swerve.utility.SwerveInputStream;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public SwerveDriveSubsystem swerve = new SwerveDriveSubsystem(Filesystem.getDeployDirectory());
  public PS4Controller controller = new PS4Controller(0);
  public boolean headingControlActive = false;

    final SwerveInputStream driveStream =
    swerve.getAngularVelocityStream(
    ()-> controller.getLeftX(), 
    ()-> -controller.getLeftY(), 
    ()-> Constants.OperatorConstants.ROTATE_AXIS)
    .withControllerHeadingAxis(
    ()-> controller.getRawAxis(3), 
    ()-> controller.getRawAxis(4))
    .withHeadingControl(()->headingControlActive)
    .withDeadband(0.05)
    .withAllianceRelativeControl();

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    SmartDashboard.putBoolean("HeadingControlActive", headingControlActive);
    //Ativa e desativa o headingcontrol
    swerve.setDefaultCommand(swerve.drive(driveStream));

    new JoystickButton(controller, 2).onTrue
    (Commands.runOnce(()-> 
    {
      System.out.println("Heading control got here");
      headingControlActive = !headingControlActive;
      SmartDashboard.putBoolean("HeadingControlActive", headingControlActive);
    }
      ).ignoringDisable(true));

    new JoystickButton(controller, Button.kCross.value).whileTrue(swerve.lockSwerve());

    //reseta o gyro
    new JoystickButton(controller, Button.kTriangle.value).onTrue(Commands.runOnce(swerve::newZero));

    //fazer o set de speeds
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
