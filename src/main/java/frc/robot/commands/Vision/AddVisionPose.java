// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.apriltag.VisionSubsytem;;

public class AddVisionPose extends Command{
  private VisionSubsytem vision;
  private Swerve swerveSubsytem;
 /** Creates a new AprilTagLocation. */
  public AddVisionPose(VisionSubsytem vision3, Swerve swerve) {
    this.vision = vision3;
    this.swerveSubsytem = swerve;
    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveSubsytem.addVisionMeasurement(vision.getVisionPose(), vision.getVisionTimestamp());
    vision.setReferencePose(swerveSubsytem.getPose());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

