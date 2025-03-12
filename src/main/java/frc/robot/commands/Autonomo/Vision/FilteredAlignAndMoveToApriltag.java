package frc.robot.commands.Autonomo.Vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Chasis;

public class FilteredAlignAndMoveToApriltag extends Command {
  private final AlignAndMoveToApriltag innerCommand;
  private final Vision vision;
  private boolean validTag = false;

  public FilteredAlignAndMoveToApriltag(Vision vision, Chasis chasis) {
    this.vision = vision;
    innerCommand = new AlignAndMoveToApriltag(vision, chasis);
    addRequirements(vision, chasis);
  }

  @Override
  public void initialize() {
    int id = vision.getIDApriltag();
    // Verifica que el ID esté entre 1 y 22, excluyendo 3,4,5,14,15 y 16
    if (id >= 1 && id <= 22 && id != 3 && id != 4 && id != 5 && id != 14 && id != 15 && id != 16) {
      validTag = true;
      innerCommand.initialize();
    } else {
      validTag = false;
    }
  }

  @Override
  public void execute() {
    if (validTag) {
      innerCommand.execute();
    }
  }

  @Override
  public boolean isFinished() {
    // Si el tag no es válido, finaliza inmediatamente;
    // de lo contrario, depende de la finalización del innerCommand.
    return !validTag || innerCommand.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    if (validTag) {
      innerCommand.end(interrupted);
    }
  }
}
