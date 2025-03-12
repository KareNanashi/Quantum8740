// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;


import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCmd extends Command {
  /** Creates a new IntakeCmd. */
  private final Intake intake;
  // private final Supplier<Double> speed;
  private final Supplier<Double> speed;
  
  public IntakeCmd(Intake intake, Supplier<Double> speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.speed = speed;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.set_speed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double speed_E = speed.get();
    if((speed_E > -0.35) && (speed_E < 0.35))
    {
        speed_E = 0;
    }
    intake.set_speed(speed_E);

    
    /* si el valor de los axes estan en un rango de -0.06 a 0.06 entonces forzarlo a 0
     * Esto ayuda a evitar el drift del motor
     */
    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.set_speed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
