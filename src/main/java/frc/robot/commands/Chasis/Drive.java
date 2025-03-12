// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chasis;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chasis;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Drive extends Command {
  /** Creates a new Drive. */
  private final Chasis chasis;
  private final double targetDistance;
  private final double speed;
  public Drive(Chasis chasis, double targetDistance, double speed) {
    // Use addRequirements()   here to declare subsystem dependencies.
    this.chasis = chasis;
    this.speed = speed;
    this.targetDistance = targetDistance;

    addRequirements(chasis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chasis.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chasis.set_motors(speed, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chasis.set_motors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (chasis.getAverageEncoderDistance()+0.1) >= targetDistance; //Return >=
  }
}
