// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.GroupMotorControllers;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  
  private DifferentialDrive driveTrain;
  private final CANSparkMax leftMotor1 = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax leftMotor2 = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax rightMotor1 = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax rightMotor2 = new CANSparkMax(4, MotorType.kBrushless);
  private final MotorControllerGroup leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2);
  private double driveRampRate = 0.5;

  private final CANSparkMax leftIntake = new CANSparkMax(6, MotorType.kBrushed);
  private final CANSparkMax rightIntake = new CANSparkMax(8, MotorType.kBrushed);
  private final MotorControllerGroup intakeMotors = new MotorControllerGroup(leftIntake, rightIntake);

  private final TalonFX leftFlyheel = new TalonFX(9);
  private final TalonFX rightFlyheel = new TalonFX(10);
  //private GroupMotorControllers flywheelMotors;

  private final XboxController xboxController = new XboxController(0);
  private RelativeEncoder leftEncoder1;
  private RelativeEncoder leftEncoder2;
  private RelativeEncoder rightEncoder1;
  private RelativeEncoder rightEncoder2;
  private AHRS gyro = new AHRS();

  private ShuffleboardTab dataTab = Shuffleboard.getTab("Data");
  private NetworkTableEntry leftEncoderPos;
  private NetworkTableEntry rightEncoderPos;
  private NetworkTableEntry gyroHeading;

  private final double DISTANCE_PER_ROTATION = 1.0d / 8.0d * 6.1d * Math.PI; // inches
  // 42 counts per revolution for the encoders

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    //SmartDashboard.putData("Auto choices", m_chooser);
    dataTab.add(m_chooser);


    rightMotor1.setInverted(true);
    driveTrain = new DifferentialDrive(leftMotors, rightMotors);

    coast();

    leftMotor1.setOpenLoopRampRate(driveRampRate);
    leftMotor2.setOpenLoopRampRate(driveRampRate);
    rightMotor1.setOpenLoopRampRate(driveRampRate);
    rightMotor2.setOpenLoopRampRate(driveRampRate);

    leftEncoder1 = leftMotor1.getEncoder();
    leftEncoder2 = leftMotor2.getEncoder();
    rightEncoder1 = rightMotor1.getEncoder();
    rightEncoder2 = rightMotor2.getEncoder();

    leftEncoder1.setPositionConversionFactor(DISTANCE_PER_ROTATION);
    leftEncoder2.setPositionConversionFactor(DISTANCE_PER_ROTATION);
    rightEncoder1.setPositionConversionFactor(DISTANCE_PER_ROTATION);
    rightEncoder2.setPositionConversionFactor(DISTANCE_PER_ROTATION);

    rightIntake.setInverted(true);
    
    rightFlyheel.setInverted(true);
    // flywheelMotors.register(rightFlyheel);
    // flywheelMotors.register(leftFlyheel);
    leftFlyheel.setNeutralMode(NeutralMode.Brake);
    rightFlyheel.setNeutralMode(NeutralMode.Brake);
    rightFlyheel.follow(leftFlyheel);


    Shuffleboard.selectTab("Data");

    leftEncoderPos = dataTab.add("Left Encoders", 0).getEntry();
    rightEncoderPos = dataTab.add("Right Encoders", 0).getEntry();
    gyroHeading = dataTab.add("Gyro Heading", 0).getEntry();


  }


  @Override
  public void robotPeriodic() {
    leftEncoderPos.setDouble((leftEncoder1.getPosition() + leftEncoder2.getPosition())/2);
    rightEncoderPos.setDouble((rightEncoder1.getPosition() + rightEncoder2.getPosition())/2);
    gyroHeading.setDouble(gyro.getYaw());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    brake();
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }


  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }


  @Override
  public void teleopInit() {
    brake();
  }

  @Override
  public void teleopPeriodic() {
    driveTrain.arcadeDrive(xboxController.getLeftY(), xboxController.getRightX());
    if (xboxController.getLeftBumper()) {
      intakeMotors.set(1);
    }
    else {
      intakeMotors.set(0);
    };

    if (xboxController.getXButton()) {
      leftFlyheel.set(ControlMode.PercentOutput, .3);
    }
    else{
      leftFlyheel.set(ControlMode.PercentOutput, 0);
    }

  }

  @Override
  public void disabledInit() {
    coast();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  public void brake() {
    leftMotor1.setIdleMode(IdleMode.kBrake);
    leftMotor2.setIdleMode(IdleMode.kBrake);
    rightMotor1.setIdleMode(IdleMode.kBrake);
    rightMotor2.setIdleMode(IdleMode.kBrake);
  }

  public void coast() {
    leftMotor1.setIdleMode(IdleMode.kCoast);
    leftMotor2.setIdleMode(IdleMode.kCoast);
    rightMotor1.setIdleMode(IdleMode.kCoast);
    rightMotor2.setIdleMode(IdleMode.kCoast);
  }
}