// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import frc.lib.Telemetry;

/** 
 * The self-contained swerve/drivetrain subsystem
 * @author Rylan Moseley
 */
public class Swerve extends SubsystemBase {
  /** Robot-mounted gyroscope (NavX or Pigeon) for field-centric driving and field positioning */
  private Pigeon2 gyro;

  // swerve module CANCoders
  private final CANCoder FL_Position = new CANCoder(5);
  private final CANCoder FR_Position = new CANCoder(6);
  private final CANCoder BL_Position = new CANCoder(7);
  private final CANCoder BR_Position = new CANCoder(8);

  // swerve module drive motors
  private final TalonFX FL_Drive = new TalonFX(11);
  private final TalonFX FR_Drive = new TalonFX(12);
  private final TalonFX BL_Drive = new TalonFX(13);
  private final TalonFX BR_Drive = new TalonFX(14);

  // swerve module azimuth (steering) motors
  private final TalonFX FL_Azimuth = new TalonFX(21);
  private final TalonFX FR_Azimuth = new TalonFX(22);
  private final TalonFX BL_Azimuth = new TalonFX(23);
  private final TalonFX BR_Azimuth = new TalonFX(24);

  // swerve module target rotations (degrees)
  private double FL_Target = 0.0;
  private double FR_Target = 0.0;
  private double BL_Target = 0.0;
  private double BR_Target = 0.0;

  // swerve module wheel speeds (percent output)
  private double FL_Speed = 0.0;
  private double FR_Speed = 0.0;
  private double BL_Speed = 0.0;
  private double BR_Speed = 0.0;

  // 'actual' read positions of each swerve module (degrees)
  private double FL_Actual_Position = 0.0;
  private double FR_Actual_Position = 0.0;
  private double BL_Actual_Position = 0.0;
  private double BR_Actual_Position = 0.0;

  // 'actual' read speeds of each swerve module (meters per second)
  private double FL_Actual_Speed = 0.0;
  private double FR_Actual_Speed = 0.0;
  private double BL_Actual_Speed = 0.0;
  private double BR_Actual_Speed = 0.0;

  // temp variables used to determine the most efficient path for each module
  private double _Path_1 = 0.0;
  private double _Path_2 = 0.0;
  private double _Path_3 = 0.0;

  // temp variables used to calculate the chassis movement
  /** B */ private double _frontTranslation = 0;
  /** A */ private double _backTranslation = 0;
  /** D */ private double _leftTranslation = 0;
  /** C */ private double _rightTranslation = 0;
  /** ROT */ private double _rotationTranslation = 0;
  /** FWD */ private double _forwardTranslation = 0;
  /** STR */ private double _sidewaysTranslation = 0;

  private double _fieldForwardTranslation = 0;
  private double _fieldSidewaysTranslation = 0;

  private long _lastRunTime = System.currentTimeMillis();
  private long _timeStep = 0;

  private double _robotForwardPosition = 0;
  private double _robotSidewaysPosition = 0;
  private double _fieldForwardPosition = 0;
  private double _fieldSidewaysPosition = 0;

  // the read yaw value from the NavX
  private double robotYaw = 0.0;

  // robot oriented / field oriented swerve drive toggle
  private boolean isRobotOriented = true;
  // length = front to back (meters)
  public static final double ROBOT_LENGTH_METERS = 0.7874;
  // width = side to side (meters)
  public static final double ROBOT_WIDTH_METERS = 0.7112;
  // wheel diameter (meters)
  public static final double WHEEL_DIAMETER_METERS = 0.1016;
  // drive gear ratio
  // TODO: check gear ratio (currently set to 'standard')
  public static final double DRIVE_GEAR_RATIO = 1/8.14;

  // constants for calculating rotation vector
  private static final double ROTATION_Y = Math.sin(Math.atan2(ROBOT_LENGTH_METERS, ROBOT_WIDTH_METERS));
  private static final double ROTATION_X = Math.cos(Math.atan2(ROBOT_LENGTH_METERS, ROBOT_WIDTH_METERS));

  private static final double DRIVE_NEUTRAL_BAND = 0.001; // TODO: tune drive motor neutral band

  private static final double DRIVE_RAMP_RATE = 0; // TODO: tune drive motor ramp rate
  
  private static final StatorCurrentLimitConfiguration DRIVE_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(); // TODO: tune drive motor stator current limit

  // encoder offsets (degrees)
  private static final double FL_ECODER_OFFSET = -223.682;
  private static final double FR_ECODER_OFFSET = -76.553;
  private static final double BL_ECODER_OFFSET = -156.006;
  private static final double BR_ECODER_OFFSET = -114.258;

  // pid values
  private static final double AZIMUTH_kP = 0.3;
  private static final double AZIMUTH_kD = 0.2;
  // calculated via JVN calculator
  // TODO check these closed-loop drive PID values
  private static final double DRIVE_kP = 0.044057;
  private static final double DRIVE_kF = 0.028998;

  // TODO check if these max speed values are reasonable and practical
  /** maximum strafe speed (meters per second) */
  private static final double MAX_LINEAR_SPEED = 4.5;
  /** maximum rotation speed (radians per second) */
  private static final double MAX_ROTATION_SPEED = Math.PI * 0.5;

  /** WPILib swerve kinematics */
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(new Translation2d(ROBOT_LENGTH_METERS/2, -ROBOT_WIDTH_METERS/2), new Translation2d(ROBOT_LENGTH_METERS/2, ROBOT_WIDTH_METERS/2), new Translation2d(-ROBOT_LENGTH_METERS/2, -ROBOT_WIDTH_METERS/2), new Translation2d(-ROBOT_LENGTH_METERS/2, ROBOT_WIDTH_METERS/2));

  private ChassisSpeeds forwardKinematics = new ChassisSpeeds();

  private ChassisSpeeds kinematicCommand = new ChassisSpeeds();

  private SwerveModuleState[] modules = new SwerveModuleState[3];

  /** Creates a new ExampleSubsystem. */
  public Swerve(Pigeon2 pigeon) {

    gyro = pigeon;

    // config drive motors
    FL_Drive.configFactoryDefault();
    FL_Drive.setInverted(TalonFXInvertType.CounterClockwise);
    FL_Drive.setNeutralMode(NeutralMode.Brake);
    FL_Drive.configNeutralDeadband(DRIVE_NEUTRAL_BAND);
    FL_Drive.configOpenloopRamp(DRIVE_RAMP_RATE);
    FL_Drive.configStatorCurrentLimit(DRIVE_CURRENT_LIMIT);
    FL_Drive.config_kP(0, DRIVE_kP);
    FL_Drive.config_kF(0, DRIVE_kF);
    
    FR_Drive.configFactoryDefault();
    FR_Drive.setInverted(TalonFXInvertType.CounterClockwise);
    FR_Drive.setNeutralMode(NeutralMode.Brake);
    FR_Drive.configNeutralDeadband(DRIVE_NEUTRAL_BAND);
    FR_Drive.configOpenloopRamp(DRIVE_RAMP_RATE);
    FR_Drive.configStatorCurrentLimit(DRIVE_CURRENT_LIMIT);
    BR_Drive.config_kP(0, DRIVE_kP);
    BR_Drive.config_kF(0, DRIVE_kF);

    BL_Drive.configFactoryDefault();
    BL_Drive.setInverted(TalonFXInvertType.CounterClockwise);
    BL_Drive.setNeutralMode(NeutralMode.Brake);
    BL_Drive.configNeutralDeadband(DRIVE_NEUTRAL_BAND);
    BL_Drive.configOpenloopRamp(DRIVE_RAMP_RATE);
    BL_Drive.configStatorCurrentLimit(DRIVE_CURRENT_LIMIT);
    BR_Drive.config_kP(0, DRIVE_kP);
    BR_Drive.config_kF(0, DRIVE_kF);

    BR_Drive.configFactoryDefault();
    BR_Drive.setInverted(TalonFXInvertType.CounterClockwise);
    BR_Drive.setNeutralMode(NeutralMode.Brake);
    BR_Drive.configNeutralDeadband(DRIVE_NEUTRAL_BAND);
    BR_Drive.configOpenloopRamp(DRIVE_RAMP_RATE);
    BR_Drive.configStatorCurrentLimit(DRIVE_CURRENT_LIMIT);
    BR_Drive.config_kP(0, DRIVE_kP);
    BR_Drive.config_kF(0, DRIVE_kF);

    // config CANcoders
    FL_Position.configFactoryDefault();
    FL_Position.configMagnetOffset(FL_ECODER_OFFSET);
    FL_Position.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    FL_Position.setPositionToAbsolute();

    FR_Position.configFactoryDefault();
    FR_Position.configMagnetOffset(FR_ECODER_OFFSET);
    FR_Position.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    FR_Position.setPositionToAbsolute();

    BL_Position.configFactoryDefault();
    BL_Position.configMagnetOffset(BL_ECODER_OFFSET);
    BL_Position.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    BL_Position.setPositionToAbsolute();

    BR_Position.configFactoryDefault();
    BR_Position.configMagnetOffset(BR_ECODER_OFFSET);
    BR_Position.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    BR_Position.setPositionToAbsolute();

    // config azimuth (steering) motors
    FL_Azimuth.configFactoryDefault();
    FL_Azimuth.setInverted(TalonFXInvertType.CounterClockwise);
    FL_Azimuth.setNeutralMode(NeutralMode.Brake);
    FL_Azimuth.configRemoteFeedbackFilter(FL_Position, 0);
    FL_Azimuth.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    FL_Azimuth.setSelectedSensorPosition(FL_Position.getAbsolutePosition());
    FL_Azimuth.config_kP(0, AZIMUTH_kP);
    FL_Azimuth.config_kD(0, AZIMUTH_kD);

    FR_Azimuth.configFactoryDefault();
    FR_Azimuth.setInverted(TalonFXInvertType.CounterClockwise);
    FR_Azimuth.setNeutralMode(NeutralMode.Brake);
    FR_Azimuth.configRemoteFeedbackFilter(FR_Position, 0);
    FR_Azimuth.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    FR_Azimuth.setSelectedSensorPosition(FR_Position.getAbsolutePosition());
    FR_Azimuth.config_kP(0, AZIMUTH_kP);
    FR_Azimuth.config_kD(0, AZIMUTH_kD);

    BL_Azimuth.configFactoryDefault();
    BL_Azimuth.setInverted(TalonFXInvertType.CounterClockwise);
    BL_Azimuth.setNeutralMode(NeutralMode.Brake);
    BL_Azimuth.configRemoteFeedbackFilter(BL_Position, 0);
    BL_Azimuth.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    BL_Azimuth.setSelectedSensorPosition(BL_Position.getAbsolutePosition());
    BL_Azimuth.config_kP(0, AZIMUTH_kP);
    BL_Azimuth.config_kD(0, AZIMUTH_kD);

    BR_Azimuth.configFactoryDefault();
    BR_Azimuth.setInverted(TalonFXInvertType.CounterClockwise);
    BR_Azimuth.setNeutralMode(NeutralMode.Brake);
    BR_Azimuth.configRemoteFeedbackFilter(BR_Position, 0);
    BR_Azimuth.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    BR_Azimuth.setSelectedSensorPosition(BR_Position.getAbsolutePosition());
    BR_Azimuth.config_kP(0, AZIMUTH_kP);
    BR_Azimuth.config_kD(0, AZIMUTH_kD);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // 'actual' read sensor positions of each module
    FL_Actual_Position = ((FL_Azimuth.getSelectedSensorPosition() / 4096) * 360) % 360;
    FR_Actual_Position = ((FR_Azimuth.getSelectedSensorPosition() / 4096) * 360) % 360;
    BL_Actual_Position = ((BL_Azimuth.getSelectedSensorPosition() / 4096) * 360) % 360;
    BR_Actual_Position = ((BR_Azimuth.getSelectedSensorPosition() / 4096) * 360) % 360;

    // 'actual' read encoder speeds per module (meters per second)
    FL_Actual_Speed = (FL_Drive.getSelectedSensorVelocity() / 4096) * 10 * DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS;
    FR_Actual_Speed = (FR_Drive.getSelectedSensorVelocity() / 4096) * 10 * DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS;
    BL_Actual_Speed = (BL_Drive.getSelectedSensorVelocity() / 4096) * 10 * DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS;
    BR_Actual_Speed = (BR_Drive.getSelectedSensorVelocity() / 4096) * 10 * DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS;

    // dashboard data
    Telemetry.setValue("drivetrain/Modules/FL/Azimuth/Target", FL_Target);
    Telemetry.setValue("drivetrain/Modules/FR/Azimuth/Target", FR_Target);
    Telemetry.setValue("drivetrain/Modules/BL/Azimuth/Target", BL_Target);
    Telemetry.setValue("drivetrain/Modules/BR/Azimuth/Target", BR_Target);
    Telemetry.setValue("drivetrain/Modules/FL/Drive/Power", FL_Speed);
    Telemetry.setValue("drivetrain/Modules/FR/Drive/Power", FR_Speed);
    Telemetry.setValue("drivetrain/Modules/BL/Drive/Power", BL_Speed);
    Telemetry.setValue("drivetrain/Modules/BR/Drive/Power", BR_Speed);
    Telemetry.setValue("drivetrain/Modules/FL/Azimuth/Actual_Position", FL_Actual_Position);
    Telemetry.setValue("drivetrain/Modules/FR/Azimuth/Actual_Position", FR_Actual_Position);
    Telemetry.setValue("drivetrain/Modules/BL/Azimuth/Actual_Position", BL_Actual_Position);
    Telemetry.setValue("drivetrain/Modules/BR/Azimuth/Actual_Position", BR_Actual_Position);
    Telemetry.setValue("drivetrain/Modules/FL/Drive/Actual_Speed", FL_Actual_Speed);
    Telemetry.setValue("drivetrain/Modules/FR/Drive/Actual_Speed", FR_Actual_Speed);
    Telemetry.setValue("drivetrain/Modules/BL/Drive/Actual_Speed", BL_Actual_Speed);
    Telemetry.setValue("drivetrain/Modules/BR/Drive/Actual_Speed", BR_Actual_Speed);
    Telemetry.setValue("drivetrain/Modules/FL/Drive/Temperature", FL_Drive.getTemperature());
    Telemetry.setValue("drivetrain/Modules/FR/Drive/Temperature", FR_Drive.getTemperature());
    Telemetry.setValue("drivetrain/Modules/BL/Drive/Temperature", BL_Drive.getTemperature());
    Telemetry.setValue("drivetrain/Modules/BR/Drive/Temperature", BR_Drive.getTemperature());
    Telemetry.setValue("drivetrain/Modules/FL/Azimuth/Temperature", FL_Azimuth.getTemperature());
    Telemetry.setValue("drivetrain/Modules/FR/Azimuth/Temperature", FR_Azimuth.getTemperature());
    Telemetry.setValue("drivetrain/Modules/BL/Azimuth/Temperature", BL_Azimuth.getTemperature());
    Telemetry.setValue("drivetrain/Modules/BR/Azimuth/Temperature", BR_Azimuth.getTemperature());
    Telemetry.setValue("drivetrain/Modules/FL/Azimuth/Bus_Voltage", FL_Azimuth.getBusVoltage());
    Telemetry.setValue("drivetrain/Modules/FR/Azimuth/Bus_Voltage", FR_Azimuth.getBusVoltage());
    Telemetry.setValue("drivetrain/Modules/BL/Azimuth/Bus_Voltage", BL_Azimuth.getBusVoltage());
    Telemetry.setValue("drivetrain/Modules/BR/Azimuth/Bus_Voltage", BR_Azimuth.getBusVoltage());
    Telemetry.setValue("drivetrain/Modules/FL/Drive/Bus_Voltage", FL_Drive.getBusVoltage());
    Telemetry.setValue("drivetrain/Modules/FR/Drive/Bus_Voltage", FR_Drive.getBusVoltage());
    Telemetry.setValue("drivetrain/Modules/BL/Drive/Bus_Voltage", BL_Drive.getBusVoltage());
    Telemetry.setValue("drivetrain/Modules/BR/Drive/Bus_Voltage", BR_Drive.getBusVoltage());
    Telemetry.setValue("drivetrain/Modules/FL/Azimuth/Output_Voltage", FL_Azimuth.getMotorOutputVoltage());
    Telemetry.setValue("drivetrain/Modules/FR/Azimuth/Output_Voltage", FR_Azimuth.getMotorOutputVoltage());
    Telemetry.setValue("drivetrain/Modules/BL/Azimuth/Output_Voltage", BL_Azimuth.getMotorOutputVoltage());
    Telemetry.setValue("drivetrain/Modules/BR/Azimuth/Output_Voltage", BR_Azimuth.getMotorOutputVoltage());
    Telemetry.setValue("drivetrain/Modules/FL/Drive/Output_Voltage", FL_Drive.getMotorOutputVoltage());
    Telemetry.setValue("drivetrain/Modules/FR/Drive/Output_Voltage", FR_Drive.getMotorOutputVoltage());
    Telemetry.setValue("drivetrain/Modules/BL/Drive/Output_Voltage", BL_Drive.getMotorOutputVoltage());
    Telemetry.setValue("drivetrain/Modules/BR/Drive/Output_Voltage", BR_Drive.getMotorOutputVoltage());
    Telemetry.setValue("drivetrain/Modules/FL/Azimuth/Supply_Current", FL_Azimuth.getSupplyCurrent());
    Telemetry.setValue("drivetrain/Modules/FR/Azimuth/Supply_Current", FR_Azimuth.getSupplyCurrent());
    Telemetry.setValue("drivetrain/Modules/BL/Azimuth/Supply_Current", BL_Azimuth.getSupplyCurrent());
    Telemetry.setValue("drivetrain/Modules/BR/Azimuth/Supply_Current", BR_Azimuth.getSupplyCurrent());
    Telemetry.setValue("drivetrain/Modules/FL/Drive/Supply_Current", FL_Drive.getSupplyCurrent());
    Telemetry.setValue("drivetrain/Modules/FR/Drive/Supply_Current", FR_Drive.getSupplyCurrent());
    Telemetry.setValue("drivetrain/Modules/BL/Drive/Supply_Current", BL_Drive.getSupplyCurrent());
    Telemetry.setValue("drivetrain/Modules/BR/Drive/Supply_Current", BR_Drive.getSupplyCurrent());
    Telemetry.setValue("drivetrain/Modules/FL/Azimuth/Stator_Current", FL_Azimuth.getStatorCurrent());
    Telemetry.setValue("drivetrain/Modules/FR/Azimuth/Stator_Current", FR_Azimuth.getStatorCurrent());
    Telemetry.setValue("drivetrain/Modules/BL/Azimuth/Stator_Current", BL_Azimuth.getStatorCurrent());
    Telemetry.setValue("drivetrain/Modules/BR/Azimuth/Stator_Current", BR_Azimuth.getStatorCurrent());
    Telemetry.setValue("drivetrain/Modules/FL/Drive/Stator_Current", FL_Drive.getStatorCurrent());
    Telemetry.setValue("drivetrain/Modules/FR/Drive/Stator_Current", FR_Drive.getStatorCurrent());
    Telemetry.setValue("drivetrain/Modules/BL/Drive/Stator_Current", BL_Drive.getStatorCurrent());
    Telemetry.setValue("drivetrain/Modules/BR/Drive/Stator_Current", BR_Drive.getStatorCurrent());
    Telemetry.setValue("drivetrain/Gyro_Temperature", gyro.getTemp());
    Telemetry.setValue("drivetrain/isRobotOriented", isRobotOriented);
    Telemetry.setValue("drivetrain/yaw", gyro.getYaw());

    _frontTranslation = ( (Math.sin(Math.toRadians(FL_Actual_Position)) * FL_Actual_Speed ) + (Math.sin(Math.toRadians(FR_Actual_Position)) * FR_Actual_Speed) ) / 2.0;
    _backTranslation = ( (Math.sin(Math.toRadians(BL_Actual_Position)) * BL_Actual_Speed ) + (Math.sin(Math.toRadians(BR_Actual_Position)) * BR_Actual_Speed) ) / 2.0;
    _leftTranslation = ( (Math.cos(Math.toRadians(FL_Actual_Position)) * FL_Actual_Speed ) + (Math.cos(Math.toRadians(BL_Actual_Position)) * BL_Actual_Speed) ) / 2.0;
    _rightTranslation = ( (Math.cos(Math.toRadians(FR_Actual_Position)) * FL_Actual_Speed ) + (Math.cos(Math.toRadians(BR_Actual_Position)) * BL_Actual_Speed) ) / 2.0;

    _rotationTranslation = ( ( (_frontTranslation - _backTranslation) / ROBOT_LENGTH_METERS ) + ( (_rightTranslation - _leftTranslation) / ROBOT_WIDTH_METERS ) ) / 2.0;
    _forwardTranslation = ( (_rotationTranslation * (ROBOT_LENGTH_METERS/2.0) + _backTranslation) + (-_rotationTranslation * (ROBOT_LENGTH_METERS/2.0) + _frontTranslation) ) / 2.0;
    _sidewaysTranslation = ( (_rotationTranslation * (ROBOT_WIDTH_METERS/2) + _rightTranslation) + (-_rotationTranslation * (ROBOT_WIDTH_METERS/2.0) + _leftTranslation) ) / 2.0;

    _fieldForwardTranslation = ( _forwardTranslation * Math.cos(Math.toRadians(gyro.getYaw())) + _sidewaysTranslation * Math.sin(Math.toRadians(gyro.getYaw())));
    _fieldSidewaysTranslation =  ( _sidewaysTranslation * Math.cos(Math.toRadians(gyro.getYaw())) - _forwardTranslation * Math.sin(Math.toRadians(gyro.getYaw())));

    Telemetry.setValue("drivetrain/kinematics/homemade/robot/forward", _forwardTranslation);
    Telemetry.setValue("drivetrain/kinematics/homemade/robot/rightward", _sidewaysTranslation);
    Telemetry.setValue("drivetrain/kinematics/homemade/clockwise_speed", _rotationTranslation);
    Telemetry.setValue("drivetrain/kinematics/homemade/field/DS_away", _fieldForwardTranslation);
    Telemetry.setValue("drivetrain/kinematics/homemade/field/DS_right", _fieldSidewaysTranslation);

    _timeStep = System.currentTimeMillis() - _lastRunTime;

    _robotForwardPosition += _forwardTranslation * (_timeStep / 1000.0);
    _robotSidewaysPosition += _sidewaysTranslation * (_timeStep / 1000.0);
    _fieldForwardPosition += _fieldForwardTranslation * (_timeStep / 1000.0);
    _fieldSidewaysPosition += _fieldSidewaysTranslation * (_timeStep / 1000.0);

    Telemetry.setValue("drivetrain/odometry/homemade/robot/forward", _robotForwardPosition);
    Telemetry.setValue("drivetrain/odometry/homemade/robot/rightward", _robotSidewaysPosition);
    Telemetry.setValue("drivetrain/odometry/homemade/field/DS_away", _fieldForwardPosition);
    Telemetry.setValue("drivetrain/odometry/homemade/field/DS_right", _fieldSidewaysPosition);

    _lastRunTime = System.currentTimeMillis();

    forwardKinematics = kinematics.toChassisSpeeds(new SwerveModuleState(FL_Actual_Speed, new Rotation2d(Math.toRadians(FL_Actual_Position))), new SwerveModuleState(FR_Actual_Speed, new Rotation2d(Math.toRadians(FR_Actual_Position))), new SwerveModuleState(BL_Actual_Speed, new Rotation2d(Math.toRadians(BL_Actual_Position))), new SwerveModuleState(BR_Actual_Speed, new Rotation2d(Math.toRadians(BR_Actual_Position))) );

    Telemetry.setValue("drivetrain/kinematics/official/robot/forward", forwardKinematics.vxMetersPerSecond);
    Telemetry.setValue("drivetrain/kinematics/official/robot/rightward", -forwardKinematics.vyMetersPerSecond);
    Telemetry.setValue("drivetrain/kinematics/official/clockwise_speed", -Math.toDegrees(forwardKinematics.omegaRadiansPerSecond));
    Telemetry.setValue("drivetrain/kinematics/official/field/DS_away", ( forwardKinematics.vxMetersPerSecond * Math.cos(Math.toRadians(gyro.getYaw())) - forwardKinematics.vyMetersPerSecond * Math.sin(Math.toRadians(gyro.getYaw()))));
    Telemetry.setValue("drivetrain/kinematics/official/field/DS_right", ( -forwardKinematics.vyMetersPerSecond * Math.cos(Math.toRadians(gyro.getYaw())) - forwardKinematics.vxMetersPerSecond * Math.sin(Math.toRadians(gyro.getYaw()))));

    Telemetry.setValue("drivetrain/odometry/official/robot/forward", 0);
    Telemetry.setValue("drivetrain/odometry/official/robot/rightward", 0);
    Telemetry.setValue("drivetrain/odometry/official/field/DS_away", 0);
    Telemetry.setValue("drivetrain/odometry/official/field/DS_right", 0);
  }

  @Override
  public void simulationPeriodic() {}

  public void joystickDrive(double LX, double LY, double RX) {

    robotYaw = gyro.getYaw();

    // WPILib swerve command
    kinematicCommand = new ChassisSpeeds(LY * MAX_LINEAR_SPEED, LX * MAX_LINEAR_SPEED, RX * MAX_ROTATION_SPEED);
    if ( !isRobotOriented ) kinematicCommand = ChassisSpeeds.fromFieldRelativeSpeeds(LY * MAX_LINEAR_SPEED, LX * MAX_LINEAR_SPEED, RX * MAX_ROTATION_SPEED, Rotation2d.fromDegrees(robotYaw));
    
    modules = kinematics.toSwerveModuleStates(kinematicCommand);

    // if joystick is idle, lock wheels to X formation to avoid pushing
    if (LX == 0 && LY == 0 && RX == 0) {
      modules[0].angle = new Rotation2d((Math.atan2( ROTATION_Y, -ROTATION_X)) % 360);
      modules[1].angle = new Rotation2d((Math.atan2( ROTATION_Y,  ROTATION_X)) % 360);
      modules[2].angle = new Rotation2d((Math.atan2(-ROTATION_Y, -ROTATION_X)) % 360);
      modules[3].angle = new Rotation2d((Math.atan2(-ROTATION_Y,  ROTATION_X)) % 360);
    }

    driveFromModuleStates(modules);
  }

  public void driveFromModuleStates ( SwerveModuleState[] modules ) {
    SwerveDriveKinematics.desaturateWheelSpeeds(modules, MAX_LINEAR_SPEED);

    FL_Target = modules[0].speedMetersPerSecond;
    FR_Target = modules[1].speedMetersPerSecond;
    BL_Target = modules[2].speedMetersPerSecond;
    BR_Target = modules[3].speedMetersPerSecond;
    FL_Speed = modules[0].angle.getDegrees();
    FR_Speed = modules[1].angle.getDegrees();
    BL_Speed = modules[2].angle.getDegrees();
    BR_Speed = modules[3].angle.getDegrees();

    // find the shortest path to an equivalent position to prevent unneccesary full rotations
    _Path_1 = Math.abs(FL_Target - FL_Actual_Position);
    _Path_2 = Math.abs((FL_Target + 360) - FL_Actual_Position);
    _Path_3 = Math.abs((FL_Target - 360) - FL_Actual_Position);
    if (Math.min(Math.min(_Path_1, _Path_2), _Path_3) == _Path_2)
      FL_Target += 360;
    if (Math.min(Math.min(_Path_1, _Path_2), _Path_3) == _Path_3)
      FL_Target -= 360;
    
    _Path_1 = Math.abs(FR_Target - FR_Actual_Position);
    _Path_2 = Math.abs((FR_Target + 360) - FR_Actual_Position);
    _Path_3 = Math.abs((FR_Target - 360) - FR_Actual_Position);
    if (Math.min(Math.min(_Path_1, _Path_2), _Path_3) == _Path_2)
      FR_Target += 360;
    if (Math.min(Math.min(_Path_1, _Path_2), _Path_3) == _Path_3)
      FR_Target -= 360;

    _Path_1 = Math.abs(BL_Target - BL_Actual_Position);
    _Path_2 = Math.abs((BL_Target + 360) - BL_Actual_Position);
    _Path_3 = Math.abs((BL_Target - 360) - BL_Actual_Position);
    if (Math.min(Math.min(_Path_1, _Path_2), _Path_3) == _Path_2)
      BL_Target += 360;
    if (Math.min(Math.min(_Path_1, _Path_2), _Path_3) == _Path_3)
      BL_Target -= 360;

    _Path_1 = Math.abs(BR_Target - BR_Actual_Position);
    _Path_2 = Math.abs((BR_Target + 360) - BR_Actual_Position);
    _Path_3 = Math.abs((BR_Target - 360) - BR_Actual_Position);
    if (Math.min(Math.min(_Path_1, _Path_2), _Path_3) == _Path_2)
      BR_Target += 360;
    if (Math.min(Math.min(_Path_1, _Path_2), _Path_3) == _Path_3)
      BR_Target -= 360;

    // correct the target positions so that they are close to the current position
    // then convert to sensor units and pass target positions to motor controllers
    FL_Azimuth.set(ControlMode.Position, ((FL_Target + (FL_Actual_Position - (FL_Actual_Position % 360))) / 360) * 4096);
    FR_Azimuth.set(ControlMode.Position, ((FR_Target + (FR_Actual_Position - (FR_Actual_Position % 360))) / 360) * 4096);
    BL_Azimuth.set(ControlMode.Position, ((BL_Target + (BL_Actual_Position - (BL_Actual_Position % 360))) / 360) * 4096);
    BR_Azimuth.set(ControlMode.Position, ((BR_Target + (BR_Actual_Position - (BR_Actual_Position % 360))) / 360) * 4096);

    // pass wheel speeds to motor controllers
    FL_Drive.set(ControlMode.Velocity, (FL_Speed/(Math.PI * WHEEL_DIAMETER_METERS)*4096)/10);
    FR_Drive.set(ControlMode.Velocity, (FR_Speed/(Math.PI * WHEEL_DIAMETER_METERS)*4096)/10);
    BL_Drive.set(ControlMode.Velocity, (BL_Speed/(Math.PI * WHEEL_DIAMETER_METERS)*4096)/10);
    BR_Drive.set(ControlMode.Velocity, (BR_Speed/(Math.PI * WHEEL_DIAMETER_METERS)*4096)/10);
  }

  /** Sets the gyroscope's current heading to 0 */
  public void zeroGyro() {
    gyro.setYaw(0);
  }

  /** toggles field/robot orientation
   * @return new isRobotOriented value
   */
  public boolean toggleRobotOrient() {
    isRobotOriented = !isRobotOriented;
    return isRobotOriented;
  }

  /** @return true if robot oriented, false if field oriented */
  public boolean getIsRobotOriented() {
    return isRobotOriented;
  }

  /** Sets the robot's orientation to robot (true) or field (false) */
  public void setRobotOriented(boolean _isRobotOriented) {
    isRobotOriented = _isRobotOriented;
  }
}
