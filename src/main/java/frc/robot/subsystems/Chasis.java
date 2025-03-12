// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Chasis extends SubsystemBase {
  /** Creates a new Chasis. */
  private final SparkMax leftBackMotor = new SparkMax(2, MotorType.kBrushless);
  private final SparkMax leftFrontMotor = new SparkMax(1, MotorType.kBrushless);
  private final SparkMax rightBackMotor = new SparkMax(5, MotorType.kBrushless);
  private final SparkMax rightFrontMotor = new SparkMax(6, MotorType.kBrushless);

  private final SparkMaxConfig configbase = new SparkMaxConfig();
  private final SparkMaxConfig rightFrontConfig = new SparkMaxConfig();
  private final SparkMaxConfig rightConfiguration = new SparkMaxConfig();

  private final SparkMaxConfig leftConfiguration = new SparkMaxConfig();
  private final SparkMaxConfig leftFrontConfiguration = new SparkMaxConfig();


  ///////////////////////Encoders y PID//////////////////////////////
  private final SparkClosedLoopController PIDLeftmotors = leftFrontMotor.getClosedLoopController();
  private final SparkClosedLoopController PIDRightmotors = rightFrontMotor.getClosedLoopController();

  private final RelativeEncoder leftencoder, rightencoder;

  private double kP = 0.00001;
  private double kI = 0;
  private double kD = 0;

  private static final double WHEEL_DIAMETER = 0.1524; // 6 inches in meters
  private static final double GEAR_RATIO = 8.45; //10.75

  private static final double DIAMETER_ROBOT = 0.5461;

  //public static final double drivingFactor = WHEEL_DIAMETER * Math.PI / GEAR_RATIO;
  /////////////////////////////////////////////////////////////////////////////////////////////////
  

  /// PID///////////
  private final ProfiledPIDController linearController;
  private static final double MAX_VELOCITY = 2.0;
  private static final double MAX_ACCELERAION = 1.0;
  /// ///////////////////////////////
  
  public Chasis() {
    configbase.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    configbase.encoder.positionConversionFactor(1).velocityConversionFactor(1); //Encoder
    configbase.closedLoop.pid(kP, kI, kD).outputRange(-0.4, 0.4);
    rightFrontConfig.apply(configbase).inverted(true);
    rightConfiguration.apply(configbase).follow(rightFrontMotor).inverted(true);
    leftConfiguration.apply(configbase).follow(leftFrontMotor);
    leftFrontConfiguration.apply(configbase);

    leftBackMotor.configure(leftConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftFrontMotor.configure(leftFrontConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightBackMotor.configure(rightConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightFrontMotor.configure(rightFrontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //////////////Encoders///////////////
    leftencoder = leftFrontMotor.getEncoder();
    rightencoder = rightFrontMotor.getEncoder();
     resetEncoder();
    ////////////////////////////////////
    
    //////////////Configure the ProfiledPIDController////////////////////// 
    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERAION);
    linearController = new ProfiledPIDController(kP, kI, kD, constraints);
    linearController.setTolerance(0.1); //10 cm tolerance
    ///////////////////////////////////////////////////////////////////
  }

  public void set_motors(double leftSpeed, double rightSpeed){
    // leftBackMotor.set(leftSpeed);
    leftFrontMotor.set(leftSpeed);
    // rightBackMotor.set(-rightSpeed);
    rightFrontMotor.set(rightSpeed);
  }

  ///////////////////////////Encoders y PID///////////////////////////
  public void setPosition(double position){
    PIDLeftmotors.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    PIDRightmotors.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public void setVelocity(double velocity){
    PIDRightmotors.setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    PIDLeftmotors.setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);

  }

  public void resetEncoder(){
    leftencoder.setPosition(0);
    rightencoder.setPosition(0);
  }

  public double getAverageEncoderDistance(){
    double leftDistance = (leftencoder.getPosition() / GEAR_RATIO) * (Math.PI * WHEEL_DIAMETER);
    double rightDistance = (rightencoder.getPosition() / GEAR_RATIO) * (Math.PI * WHEEL_DIAMETER);

    System.out.println("Left Encoder (rotations): " + leftencoder.getPosition());
    System.out.println("Right Encoder (rotations): " + rightencoder.getPosition());
    System.out.println("Left Distance (meters): " + leftDistance);
    System.out.println("Right Distance (meters): " + rightDistance);
    
    return (leftDistance + rightDistance) / 2.0;
  }

  public double driveToDistance(double targetDistance){
    double current = getAverageEncoderDistance();
    double output = linearController.calculate(current, targetDistance);

    //Clamp output
    output = Math.max(-1.0, Math.min(1.0, output));

    set_motors(output, output);

    return Math.abs(targetDistance-current);
  }

  public boolean atSetpoint(){
    return linearController.atSetpoint();
  }


  //Funcion rotar

  // public double degree45(){
  //   return 8;
  // }
  public double rotacion(double degree){
    
    double distance_rot90_chasis = (Math.PI * DIAMETER_ROBOT)/(degree);
    //double rueda_rot = (Math.PI * WHEEL_DIAMETER)/(degree);

    // double rot = distance_rot90_chasis/rueda_rot;
    //boolean rot = rueda_rot >= distance_rot90_chasis;

    return distance_rot90_chasis;
  }

  public boolean rot(double rot, double targetDistance){
    resetEncoder();
    return targetDistance >= rot;
    //20cm 
    //1cm 2cm 3cm 10 19 21
  }//90 -> rueda_rot >= distance_rot

  ///45° -> 8
  /// 90° -> 4
  /// 180° -> 2
  /// 60° -> 6
  /// 30° -> 12
  /// 120° -> 3
  /// 240° -> 1.5
  /// 
  public double get_left_encoder_distance(){
    // return leftencoder.getPosition();
    double leftDistance = (leftencoder.getPosition() / GEAR_RATIO) * (Math.PI * WHEEL_DIAMETER);
    return leftDistance;
  }

  public double get_right_encoder_distance(){
    double rightDistance = (rightencoder.getPosition() / GEAR_RATIO) * (Math.PI * WHEEL_DIAMETER);
    return rightDistance;
  }
  
  //////////////////////////////////////////////////////////////////// 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /////////////////////Encoders y PID///////////////////////
    SmartDashboard.putNumber("Actual Position RE: ", rightencoder.getPosition());
    SmartDashboard.putNumber("Actual Position LE: ", leftencoder.getPosition());
    SmartDashboard.putNumber("Distance", getAverageEncoderDistance());
    
    SmartDashboard.putNumber("Left Encoder: ", get_left_encoder_distance());
    SmartDashboard.putNumber("Right Encoder: ", get_right_encoder_distance());

    if (SmartDashboard.getBoolean("Reset Encoder", false)){
      SmartDashboard.putBoolean("Reset Encoder", false);
      resetEncoder();
    }
  }

}
