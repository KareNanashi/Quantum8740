package frc.robot.commands.Elevador;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class MoveElevatorToPosition extends Command {
    private final Elevator elevator;
    private final double targetPosition;

    public MoveElevatorToPosition(Elevator elevator, double targetPosition) {
        this.elevator = elevator;
        this.targetPosition = targetPosition;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.ElevatorGoPosition(targetPosition); // Establecer la posición objetivo
    }

    @Override
    public void execute() {
        // No es necesario hacer nada en execute, ya que el PID controla el movimiento
        elevator.ElevatorGoPosition(targetPosition); // Establecer la posición objetivo

    }

    @Override
    public boolean isFinished() {
        // Verificar si el elevador está dentro de la tolerancia de ±2 cm
        double currentPosition = elevator.getCurrentPosition();
    System.out.println("Elevador posición actual: " + currentPosition + ", objetivo: " + targetPosition);
    return Math.abs(currentPosition - targetPosition) < 2;  // Asegúrate de que la tolerancia sea adecuada
    }

    @Override
    public void end(boolean interrupted) {
    }
}