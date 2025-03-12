// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chasis;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chasis;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotarIzquierda extends Command {
  /** Creates a new RotarIzquierda. */
  private final Chasis chasis;
  private final double right_speed;
  private final double left_speed;
  private final double degree;
  public RotarIzquierda(Chasis chasis, double right_speed, double left_speed, double degree) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chasis = chasis;
    this.right_speed = right_speed;
    this.left_speed = left_speed;
    this.degree = degree;
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
    chasis.set_motors(left_speed, right_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // chasis.set_motors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentDistance = chasis.get_right_encoder_distance();
    double targetDistance = chasis.rotacion(degree);
  return currentDistance >= targetDistance;
    // return chasis.rot(chasis.get_right_encoder_distance(), chasis.rotacion(degree));
  }
}
