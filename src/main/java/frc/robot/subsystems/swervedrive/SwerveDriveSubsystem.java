// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import java.io.File;
import java.util.function.DoubleSupplier;

import com.studica.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.parser.SwerveParser;
import yams.mechanisms.config.SwerveDriveConfig;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.utility.SwerveInputStream;
import yams.telemetry.SwerveDriveTelemetryConfig;

public class SwerveDriveSubsystem extends SubsystemBase {

  private SwerveDrive swerveDrive;
  private AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
  private double vx = 0.0;
  private double vy = 0.0;
  private double vrot = 0.0;
  
  private final StructPublisher<Pose2d> posePublisher =
    NetworkTableInstance.getDefault()
        .getStructTopic("RobotPose", Pose2d.struct)
        .publish();

  private Pose2d startPose = new Pose2d(
  new Translation2d(
  Meter.of(1), 
  Meter.of(4)
  ),Rotation2d.fromDegrees(0));


  public SwerveDriveSubsystem(File directory) {
  SmartDashboard.putData(this);
  SmartDashboard.putData(gyro);
  SmartDashboard.setDefaultNumber("speeds/vx", vx);
  SmartDashboard.setDefaultNumber("speeds/vy", vy);
  SmartDashboard.setDefaultNumber("speeds/vrot", vrot);

  var cfg = new SwerveDriveConfig()
  .withStartingPose(startPose)
  .withSubsystem(this)
  .withGyro(()-> gyro.getRotation2d().getMeasure())
  .withTranslationController(new PIDController(0, 0, 0))
  .withMaximumChassisSpeed(MetersPerSecond.of(Constants.OperatorConstants.MAX_SPEED), RadiansPerSecond.of(Math.PI * 2))
  .withRotationController(new PIDController(0, 0, 0))
  .withTelemetry("swerve", new SwerveDriveTelemetryConfig(yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity.HIGH));

  SwerveParser.parse(new File(Filesystem.getDeployDirectory(), "swerve/base"));

  
  swerveDrive = SwerveParser.createSwerveDrive(cfg);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getPose();
    swerveDrive.updateTelemetry();
    
  }

  public SwerveInputStream getAngularVelocityStream(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot){
    return new SwerveInputStream(swerveDrive, x, y, rot);
  }

  public Rotation2d getHeading(){
    return new Rotation2d(swerveDrive.getGyroAngle());
  }

  public Pose2d getPose(){
    return swerveDrive.getPose();
  }

  public void resetOdometry(){
    swerveDrive.resetOdometry(startPose);
  }

  public Field2d getField2d(){
    return swerveDrive.getField2d();
  }

  public Command drive(SwerveInputStream stream){
    return swerveDrive.drive(
      ()-> ChassisSpeeds.fromFieldRelativeSpeeds
      (stream.get(), 
      new Rotation2d(
        swerveDrive.getGyroAngle()
        )));
  }

  public Command driveToPointYAMS(Pose2d pt){
    return swerveDrive.driveToPose(pt);
  }

  public Command newZero(){
    return Commands.runOnce(swerveDrive::zeroGyro, this).withName("Swerve");
  }

  public Rotation2d getAngle(){
    return new Rotation2d(swerveDrive.getGyroAngle());
  }

  public Command lockSwerve(){
    return this.run(()-> swerveDrive.lockPose());
  }

  public Command setSpeedsFromDashboard(){
    return swerveDrive.drive(()->
     new ChassisSpeeds(
      SmartDashboard.getNumber("speeds/vx", vx), 
      SmartDashboard.getNumber("speeds/vy", vy), 
      SmartDashboard.getNumber("speeds/vrot", vrot)));
  }

  @Override
  public void simulationPeriodic() {
    swerveDrive.simIterate();
    posePublisher.set(swerveDrive.getPose());
    
    swerveDrive.updateTelemetry();
    // This method will be called once per scheduler run during simulation
  }
}
