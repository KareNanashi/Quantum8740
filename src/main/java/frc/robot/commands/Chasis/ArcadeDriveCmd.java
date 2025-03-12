// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Chasis;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chasis;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArcadeDriveCmd extends Command {
  /** Creates a new ArcadeDriveCmd. */
  private final Chasis chasis;
  private final Supplier<Double> speedFunction, turnFunction;
  public ArcadeDriveCmd(Chasis chasis, Supplier<Double> speedFunction, Supplier<Double> turnFunction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chasis = chasis;
    this.speedFunction = speedFunction;
    this.turnFunction = turnFunction;
    addRequirements(chasis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chasis.set_motors(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double realTimeSpeed = speedFunction.get();
    double realTimeRun = turnFunction.get();

    double left = realTimeSpeed+realTimeRun;
    double right = realTimeSpeed - realTimeRun;
    /* si el valor de los axes estan en un rango de -0.06 a 0.06 entonces forzarlo a 0
     * Esto ayuda a evitar el drift del motor
     */
    if((left > -0.26) && (left < 0.26))
    {
        left = 0;
    }
    /* si el valor de los axes estan en un rango de -0.06 a 0.06 entonces forzarlo a 0
     * Esto ayuda a evitar el drift del motor
     */
    if((right > -0.26) && (right < 0.26))
    {
        right = 0;
    }
    chasis.set_motors(left, right);
    // double left = leftSpeed.get();
    // double right= rightSpeed.get();

    // chasis.set_motors(left, right);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chasis.set_motors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
