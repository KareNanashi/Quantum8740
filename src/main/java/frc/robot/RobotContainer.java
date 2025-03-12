// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autonomo.BlueApriltag;
import frc.robot.commands.Autonomo.RedApriltag;
import frc.robot.commands.Autonomo.Vision.AlignAndMoveToApriltag;
import frc.robot.commands.Autonomo.Vision.FilteredAlignAndMoveToApriltag;
import frc.robot.commands.Chasis.ArcadeDriveCmd;
import frc.robot.commands.Chasis.Drive;
import frc.robot.commands.Chasis.RotarDerecha;
import frc.robot.commands.Chasis.RotarIzquierda;
import frc.robot.commands.Elevador.ElevatorCmd;
import frc.robot.commands.Elevador.ElevatorIntakeSequence;
import frc.robot.commands.Elevador.MoveElevatorToPosition;
import frc.robot.commands.Elevador.ResetEncoders;
import frc.robot.commands.Garra.GarraCmd;
import frc.robot.commands.Intake.IntakeCmd;
import frc.robot.commands.Intake.IntakeResetEncoders;
import frc.robot.commands.Intake.IntakeSetPositionCmd;
import frc.robot.subsystems.Chasis;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Garra;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;

import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

// import java.nio.channels.NonWritableChannelException;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.filter.SlewRateLimiter;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Chasis chasis = new Chasis();
  private final Elevator elevator = new Elevator();
  private final Garra garra = new Garra();
  private final Intake intake = new Intake();
  private final Vision vision = new Vision();
  private final XboxController control_1 = new XboxController(0);
  private final XboxController control_2 = new XboxController(1);
  private final Vision phVision = new Vision();
  DigitalInput downlimitswitch = new DigitalInput(0);
  DigitalInput uplimitswitch = new DigitalInput(1);
  private final Command MoverAdelante = new Drive(chasis, 2, 0.4);
  private final Command Auto_1Coral = new Drive(chasis, 2, 0.4).andThen(new MoveElevatorToPosition(elevator, 175)).andThen(new GarraCmd(garra, -0.5, -0.5));
  SendableChooser<Command> mChooser = new SendableChooser<>();
  SlewRateLimiter filter = new SlewRateLimiter(0.5);
  
  public RobotContainer() {
    // Configure the trigger bindings
    final double ELEVATOR_RAISED_THRESHOLD = 100; // Ajusta este valor según tus mediciones
    final double SLOW_FACTOR = 0.5; // Factor para reducir la velocidad cuando el elevador está elevado
    configureBindings();
    // chasis.setDefaultCommand(new ArcadeDriveCmd(chasis, () -> -control_1.getRawAxis(1) , () -> control_1.getRawAxis(4)));
    chasis.setDefaultCommand(new ArcadeDriveCmd(
    chasis,
    () -> {
      double speed = -control_1.getRawAxis(1);
      // Si el elevador está por encima del umbral, reduce la velocidad
      
      if (elevator.getCurrentPosition() > ELEVATOR_RAISED_THRESHOLD) {
        speed *= SLOW_FACTOR;
      }
      // return speed*0.4;
      return filter.calculate(speed);
    },
    () -> {
      double turn = control_1.getRawAxis(4);
      if (elevator.getCurrentPosition() > ELEVATOR_RAISED_THRESHOLD) {
        turn *= SLOW_FACTOR;
      }
      return turn*0.4;
    }
  ));
    SmartDashboard.putData(CommandScheduler.getInstance());
    // startBreathingEffect();
    intake.setDefaultCommand(new IntakeCmd(intake,()-> (-control_2.getRawAxis(1)*0.4)));
  }

  private void configureBindings() {

    //CONTROL 1
    // new JoystickButton(control_1, 3).whileTrue(new IntakeCmd(intake, 0.6)).whileFalse(new IntakeCmd(intake, 0));
    // new JoystickButton(control_1, 4).whileTrue(new IntakeCmd(intake, -0.6)).whileFalse(new IntakeCmd(intake, 0));
    // new JoystickButton(control_1, 5).whileTrue(new GarraCmd(garra, -0.5,0.5)).whileFalse(new GarraCmd(garra, 0,0));
    // new JoystickButton(control_1, 6).whileTrue(new GarraCmd(garra, 0.5,-0.5)).whileFalse(new GarraCmd(garra, 0,0));

    new JoystickButton(control_1, 1).whileTrue(new ElevatorCmd(elevator, 0.75).unless(() -> downlimitswitch.get() == false));
    new JoystickButton(control_1, 2).whileTrue(new ElevatorCmd(elevator, -0.75).unless(() -> uplimitswitch.get() == false));
    // new Trigger(() -> control_1.getRawButton(3)).onTrue(new IntakeSetPositionCmd(intake, 40));

    // new Trigger(()-> control_1.getRawButton(3)).onTrue(new MoveElevatorToPosition(elevator, 0)); //Primera alga
    // new Trigger(()-> control_1.getRawButton(4)).onTrue(new MoveElevatorToPosition(elevator, 0)); //Segunda alga
    // new Trigger(()-> control_1.getRawButton(5)).onTrue(new MoveElevatorToPosition(elevator, 0)); //Coral Station
    
    //CONTROL 2

    // new Trigger(() -> control_2.getRawButton(6))
    // .onTrue(new FilteredAlignAndMoveToApriltag(vision, chasis));
    new JoystickButton(control_2, 5).whileTrue(new GarraCmd(garra, -0.5,-0.6)).whileFalse(new GarraCmd(garra, 0,0));
    new JoystickButton(control_2, 6).whileTrue(new GarraCmd(garra, 0.5,0.6)).whileFalse(new GarraCmd(garra, 0,0));

    new Trigger(() -> control_2.getRawButton(3))
    .onTrue(new MoveElevatorToPosition(elevator, 260));

    new Trigger(() -> control_2.getRawButton(2))
    .onTrue(new MoveElevatorToPosition(elevator, 0));

    new Trigger(() -> control_2.getRawButton(1))
    .onTrue(new MoveElevatorToPosition(elevator, 110));

    new Trigger(() -> control_2.getRawButton(4))
    .onTrue(new MoveElevatorToPosition(elevator, 331));

    // new Trigger(() -> control_2.getRawButton(5))
    // .onTrue(new ElevatorIntakeSequence(elevator, intake, 331, 100));

    new JoystickButton(control_2, 10).onTrue(new ResetEncoders(elevator));
    // new JoystickButton(control_2, 10).onTrue(new IntakeResetEncoders(intake));

    
  
  }

  
  public Command getAutonomousCommand() {
    // return new Drive(chasis, 2, 0.4).andThen(new IntakeSetPositionCmd(intake,100)).andThen(new MoveElevatorToPosition(elevator, 175)).andThen(new IntakeSetPositionCmd(intake, 20)).andThen(new GarraCmd(garra, -0.5, -0.5));    
    return new Drive(chasis, 1.80, 0.4).
    andThen(new IntakeSetPositionCmd(intake, 40)).
    andThen(new MoveElevatorToPosition(elevator, 330)).
    andThen(new IntakeSetPositionCmd(intake, 10)).
    andThen(new WaitCommand(1)).
    andThen(new Drive(chasis, 0.47, 0.05)).
    andThen(new GarraCmd(garra, 0.5, 0.5));
  }

}

