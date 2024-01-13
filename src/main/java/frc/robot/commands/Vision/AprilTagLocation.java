// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.apriltag.AprilTagVision3;;

public class AprilTagLocation extends Command{
  private AprilTagVision3 vision;
 /** Creates a new AprilTagLocation. */
  public AprilTagLocation(AprilTagVision3 vision3) {
    this.vision = vision3;
    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println(vision.returnYaw());

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

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

