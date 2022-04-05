// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.GroupMotorControllers;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.LedStrip.ColorChoices;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;

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

  private final CANSparkMax leftIntake = new CANSparkMax(7, MotorType.kBrushed);
  private final CANSparkMax rightIntake = new CANSparkMax(8, MotorType.kBrushed);
  private final MotorControllerGroup intakeMotors = new MotorControllerGroup(leftIntake, rightIntake);

  private final TalonFX leftFlywheel = new TalonFX(9);
  private final TalonFX rightFlywheel = new TalonFX(10);
  //private GroupMotorControllers flywheelMotors;

  private final TalonFX leftClimber = new TalonFX(11);
  private final TalonFX rightClimber = new TalonFX(12);

  private final XboxController xboxController = new XboxController(0);
  private RelativeEncoder leftEncoder1;
  private RelativeEncoder leftEncoder2;
  private RelativeEncoder rightEncoder1;
  private RelativeEncoder rightEncoder2;
  private AHRS gyro = new AHRS();

  private ShuffleboardTab dataTab = Shuffleboard.getTab("Data");
  private ShuffleboardTab LEDTab = Shuffleboard.getTab("LEDs");
  private NetworkTableEntry leftEncoderPos;
  private NetworkTableEntry rightEncoderPos;
  private NetworkTableEntry gyroHeading;
  private NetworkTableEntry flywheelSpeedSlider;
  private double flywheelSpeed;
  private UsbCamera camera;
  private LedStrip lightStrip;
  // private double voltage;
  private NetworkTableEntry voltage;
  private NetworkTableEntry lightMode;
  private final SendableChooser<ColorChoices> colorChoicer = new SendableChooser<>();
  private ComplexWidget colorChoosereee;
  private boolean lastChoice;
//  private PowerDistribution PDP = new PowerDistribution();
  
  private final double DISTANCE_PER_ROTATION = 1.0d / 8.0d * 6.1d * Math.PI; // inches
  // 42 counts per revolution for the encoders

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    //SmartDashboard.putData("Auto choices", m_chooser);
    dataTab.add(m_chooser).withSize(2, 1);

    //camera = CameraServer.startAutomaticCapture();
    
    leftMotor1.restoreFactoryDefaults();
    leftMotor2.restoreFactoryDefaults();
    rightMotor1.restoreFactoryDefaults();
    rightMotor2.restoreFactoryDefaults();
    leftIntake.restoreFactoryDefaults();
    rightIntake.restoreFactoryDefaults();
    leftFlywheel.configFactoryDefault();
    rightFlywheel.configFactoryDefault();
    leftClimber.configFactoryDefault();
    rightClimber.configFactoryDefault();
    
    coast();

    leftMotor1.setOpenLoopRampRate(driveRampRate);
    leftMotor2.setOpenLoopRampRate(driveRampRate);
    rightMotor1.setOpenLoopRampRate(driveRampRate);
    rightMotor2.setOpenLoopRampRate(driveRampRate);
    
    rightMotors.setInverted(true);
    driveTrain = new DifferentialDrive(leftMotors, rightMotors);

    leftEncoder1 = leftMotor1.getEncoder();
    leftEncoder2 = leftMotor2.getEncoder();
    rightEncoder1 = rightMotor1.getEncoder();
    rightEncoder2 = rightMotor2.getEncoder();

    leftEncoder1.setPositionConversionFactor(DISTANCE_PER_ROTATION);
    leftEncoder2.setPositionConversionFactor(DISTANCE_PER_ROTATION);
    rightEncoder1.setPositionConversionFactor(DISTANCE_PER_ROTATION);
    rightEncoder2.setPositionConversionFactor(DISTANCE_PER_ROTATION);

    rightIntake.setInverted(true);
    leftIntake.setInverted(true);
    rightIntake.setIdleMode(IdleMode.kCoast);
    leftIntake.setIdleMode(IdleMode.kCoast);
    
    rightFlywheel.setInverted(true);
    // flywheelMotors.register(rightFlyheel);
    // flywheelMotors.register(leftFlyheel);
    leftFlywheel.setNeutralMode(NeutralMode.Coast);
    rightFlywheel.setNeutralMode(NeutralMode.Coast);
    rightFlywheel.follow(leftFlywheel);

    rightClimber.setInverted(true);
    leftClimber.setNeutralMode(NeutralMode.Brake);
    rightClimber.setNeutralMode(NeutralMode.Brake);
    rightClimber.follow(leftClimber);

    Shuffleboard.selectTab("Data");

    leftEncoderPos = dataTab.add("Left Encoders", 0).getEntry();
    rightEncoderPos = dataTab.add("Right Encoders", 0).getEntry();
    gyroHeading = dataTab.add("Gyro Heading", 0).withWidget(BuiltInWidgets.kGyro).getEntry();
    lightMode = LEDTab.add("LED Control Mode", true).withWidget(BuiltInWidgets.kToggleSwitch).getEntry();
    voltage = LEDTab.add("LED Voltage", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 4)).getEntry();
    boolean firstTry = true;
    for (ColorChoices choice: ColorChoices.values()) {
      if (firstTry) {
        colorChoicer.setDefaultOption(choice.toString(), choice);
        firstTry = false;
      }
      else {
        colorChoicer.addOption(choice.toString(), choice);

      }
    }
    colorChoosereee = LEDTab.add(colorChoicer).withSize(2, 1).withWidget(BuiltInWidgets.kComboBoxChooser);

    lastChoice = lightMode.getBoolean(true);

    //NetworkTableEntry pdpview = dataTab.add("PDP", 0).withWidget(BuiltInWidgets.kPowerDistribution).getEntry();
    //TODO
    //dataTab.addCamera("Front View", camera.getName(), "mjpg:http://0.0.0.0:1181/?action=stream");
    //dataTab.add(camera);

    flywheelSpeedSlider = dataTab.add("Flywheel Speed", .5)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0, "max", 1))
    .getEntry();

    DriverStation.getAlliance();

    lightStrip = new LedStrip(0);

  }


  @Override
  public void robotPeriodic() {
    leftEncoderPos.setDouble((leftEncoder1.getPosition() + leftEncoder2.getPosition())/2);
    rightEncoderPos.setDouble((rightEncoder1.getPosition() + rightEncoder2.getPosition())/2);
    gyroHeading.setDouble(gyro.getYaw());
    flywheelSpeed = flywheelSpeedSlider.getDouble(0);
    //TODO
    /*if (lastChoice != lightMode.getBoolean(true)) {
      if (lightMode.getBoolean(true)) {
        //voltage.delete();
        //LEDTab.add(colorChoicer).withSize(2, 1).withWidget(BuiltInWidgets.kComboBoxChooser);
      }
      else {
        //colorChoicer.close();
        //voltage = LEDTab.add("LED Voltage", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 4)).getEntry();
      }
    }*/
    if (lightMode.getBoolean(true)) {
      lightStrip.displayColor(colorChoicer.getSelected());
    }
    else {
      lightStrip.displayColor(voltage.getDouble(0));
      
    }

    lastChoice = lightMode.getBoolean(true);
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
    driveTrain.arcadeDrive(xboxController.getLeftY(), -xboxController.getRightX());
    if (xboxController.getLeftBumper()) {
      intakeMotors.set(1);
    }
    else {
      intakeMotors.set(0);
    }

    // if (xboxController.getRightBumper()) {
    //   rightIntake.set(1);
    // }
    // else {
    //   rightIntake.set(0);
    // }

    if (xboxController.getXButton()) {
      leftFlywheel.set(TalonFXControlMode.PercentOutput, flywheelSpeed);
    }
    else {
      leftFlywheel.set(TalonFXControlMode.PercentOutput, 0);
    }

    if (xboxController.getBButton()) {
      leftClimber.set(TalonFXControlMode.PercentOutput, -.50);
    }
    else if (xboxController.getAButton()) {
      leftClimber.set(TalonFXControlMode.PercentOutput, .50);
    }
    else {
      leftClimber.set(TalonFXControlMode.PercentOutput, 0);
    };

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
    leftMotor2.setIdleMode(IdleMode.kCoast);
    rightMotor1.setIdleMode(IdleMode.kBrake);
    rightMotor2.setIdleMode(IdleMode.kCoast);
  }

  public void coast() {
    leftMotor1.setIdleMode(IdleMode.kCoast);
    leftMotor2.setIdleMode(IdleMode.kCoast);
    rightMotor1.setIdleMode(IdleMode.kCoast);
    rightMotor2.setIdleMode(IdleMode.kCoast);
  }
}