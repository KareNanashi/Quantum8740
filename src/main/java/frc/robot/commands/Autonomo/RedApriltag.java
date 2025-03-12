// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomo;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chasis;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RedApriltag extends Command {
  /** Creates a new RedApriltag. */
  private final Vision vision;
  private final Chasis chasis;
  private final PIDController tuController;
  private final PIDController dController;
  private double targetDistance;
  private double initialEncoderDistance;
  Integer id_2 = 2;
  Integer id_8 = 8;

  public RedApriltag(Chasis chasis, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.vision = vision;
    this.chasis = chasis;
    this.tuController = new PIDController(0.1, 0, 0);
    this.dController = new PIDController(0.1, 0, 0);
    addRequirements(chasis, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetDistance = vision.getXAxis();
    initialEncoderDistance = chasis.getAverageEncoderDistance();
    tuController.setSetpoint(0);
    dController.setSetpoint(targetDistance - 0.3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (vision.getIDApriltag() == id_2 || vision.getIDApriltag() == id_8){
      double yawError = vision.getZAngle();
      double turnOutput = tuController.calculate(yawError);
      chasis.set_motors(turnOutput,-turnOutput);

      //Acercarse
      double currentDistance = chasis.getAverageEncoderDistance() - initialEncoderDistance;
      double distanceError = targetDistance - currentDistance;
      double driveOutput = dController.calculate(distanceError);
      chasis.set_motors(driveOutput, driveOutput);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chasis.set_motors(0,0);
    chasis.resetEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentDistance = chasis.getAverageEncoderDistance() - initialEncoderDistance;
    return Math.abs(vision.getZAngle()) < 1.0 && Math.abs(targetDistance-currentDistance) <0.1;
  }
}
