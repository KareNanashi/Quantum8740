// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevador;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCmd extends Command {
  /** Creates a new ElevatorCmd. */
  private final Elevator elevator;
  // private final Supplier<Double> speed;
  private final double speed;
  
  public ElevatorCmd(Elevator elevator, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.speed = speed;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.set_speed(0);
    elevator.reset_encoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double speed_E = speed.get();
    elevator.set_speed(speed);
    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.set_speed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
