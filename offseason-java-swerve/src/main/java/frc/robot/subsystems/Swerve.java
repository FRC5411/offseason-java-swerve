// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.kauailabs.navx.frc.AHRS;
import frc.lib.Telemetry;

public class Swerve extends SubsystemBase {
  /** Robot-mounted gyroscope (NavX or Pigeon) for field-centric driving and field positioning */
  private AHRS gyro;

  // swerve module CANCoders
  private final CANCoder FL_Position = new CANCoder(5);
  private final CANCoder FR_Position = new CANCoder(6);
  private final CANCoder BL_Position = new CANCoder(7);
  private final CANCoder BR_Position = new CANCoder(8);

  // swerve module drive motors
  private final TalonFX FL_Drive = new TalonFX(11);
  private final TalonFX FR_Drive = new TalonFX(12);
  private final TalonFX BL_Drive = new TalonFX(12);
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

  // swerve module vector components
  private double FL_X = 0.0;
  private double FR_X = 0.0;
  private double BL_X = 0.0;
  private double BR_X = 0.0;
  private double FL_Y = 0.0;
  private double FR_Y = 0.0;
  private double BL_Y = 0.0;
  private double BR_Y = 0.0;

  // 'actual' read positions of each swerve module (degrees)
  private double FL_Actual = 0.0;
  private double FR_Actual = 0.0;
  private double BL_Actual = 0.0;
  private double BR_Actual = 0.0;

  // temp variables used to determine the most efficient path for each module
  private double _Path_1 = 0.0;
  private double _Path_2 = 0.0;
  private double _Path_3 = 0.0;

  // swerve wheel speed ratio preservation while capping speeds at 100%
  private double _speedRegulator = 1.0;

  // the read yaw value from the NavX
  private double robotYaw = 0.0;
  // the rotation offset of the NavX to the robot (degrees)
  private static final double GYRO_OFFSET = 0.0;

  // robot oriented / field oriented swerve drive toggle
  private boolean isRobotOriented = true;

  // length = front to back (inches)
  public static final double ROBOT_LENGTH = 31.0;
  // width = side to side (inches)
  public static final double ROBOT_WIDTH = 28.0;

  // constants for calculating rotation vector
  private static final double ROTATION_Y = Math.sin(Math.atan2(ROBOT_LENGTH, ROBOT_WIDTH));
  private static final double ROTATION_X = Math.cos(Math.atan2(ROBOT_LENGTH, ROBOT_WIDTH));

  private static final double DRIVE_NEUTRAL_BAND = 0.001; // TODO: tune drive motor neutral band
  private static final double AZIMUTH_NEUTRAL_BAND = 0.001; // TODO: tune azimuth motor neutral band

  private static final double DRIVE_RAMP_RATE = 0; // TODO: tune drive motor ramp rate


  // encoder offsets (degrees)
  // TODO: measure encoder offsets
  private static final int FL_ECODER_OFFSET = 0;
  private static final int FR_ECODER_OFFSET = 0;
  private static final int BL_ECODER_OFFSET = 0;
  private static final int BR_ECODER_OFFSET = 0;

  // pid values
  // TODO get PID values from other code
  private static final double AZIMUTH_kP = 0;
  private static final double AZIMUTH_kI = 0;
  private static final double AZIMUTH_kD = 0;
  private static final double AZIMUTH_INTEGRAL_ZONE = 0;

  /** Creates a new ExampleSubsystem. */
  public Swerve(AHRS NavX) {

    gyro = NavX;

    // config drive motors
    // todo config current limits to prevent overheating
    FL_Drive.configFactoryDefault();
    FL_Drive.setInverted(TalonFXInvertType.CounterClockwise);
    FL_Drive.setNeutralMode(NeutralMode.Brake);
    FL_Drive.configNeutralDeadband(DRIVE_NEUTRAL_BAND);
    FL_Drive.configOpenloopRamp(DRIVE_RAMP_RATE);

    FR_Drive.configFactoryDefault();
    FR_Drive.setInverted(TalonFXInvertType.CounterClockwise);
    FR_Drive.setNeutralMode(NeutralMode.Brake);
    FR_Drive.configNeutralDeadband(DRIVE_NEUTRAL_BAND);
    FR_Drive.configOpenloopRamp(DRIVE_RAMP_RATE);

    BL_Drive.configFactoryDefault();
    BL_Drive.setInverted(TalonFXInvertType.CounterClockwise);
    BL_Drive.setNeutralMode(NeutralMode.Brake);
    BL_Drive.configNeutralDeadband(DRIVE_NEUTRAL_BAND);
    BL_Drive.configOpenloopRamp(DRIVE_RAMP_RATE);

    BR_Drive.configFactoryDefault();
    BR_Drive.setInverted(TalonFXInvertType.CounterClockwise);
    BR_Drive.setNeutralMode(NeutralMode.Brake);
    BR_Drive.configNeutralDeadband(DRIVE_NEUTRAL_BAND);
    BR_Drive.configOpenloopRamp(DRIVE_RAMP_RATE);

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
    FL_Azimuth.configNeutralDeadband(AZIMUTH_NEUTRAL_BAND);
    FL_Azimuth.setNeutralMode(NeutralMode.Brake);
    FL_Azimuth.configRemoteFeedbackFilter(FL_Position, 0);
    FL_Azimuth.setSelectedSensorPosition(FL_Position.getAbsolutePosition());
    FL_Azimuth.config_kP(0, AZIMUTH_kP);
    FL_Azimuth.config_kI(0, AZIMUTH_kI);
    FL_Azimuth.config_kD(0, AZIMUTH_kD);
    FL_Azimuth.config_IntegralZone(0, AZIMUTH_INTEGRAL_ZONE);

    FR_Azimuth.configFactoryDefault();
    FR_Azimuth.setInverted(TalonFXInvertType.CounterClockwise);
    FR_Azimuth.configNeutralDeadband(AZIMUTH_NEUTRAL_BAND);
    FR_Azimuth.setNeutralMode(NeutralMode.Brake);
    FR_Azimuth.configRemoteFeedbackFilter(FR_Position, 0);
    FR_Azimuth.setSelectedSensorPosition(FR_Position.getAbsolutePosition());
    FR_Azimuth.config_kP(0, AZIMUTH_kP);
    FR_Azimuth.config_kI(0, AZIMUTH_kI);
    FR_Azimuth.config_kD(0, AZIMUTH_kD);
    FR_Azimuth.config_IntegralZone(0, AZIMUTH_INTEGRAL_ZONE);

    BL_Azimuth.configFactoryDefault();
    BL_Azimuth.setInverted(TalonFXInvertType.CounterClockwise);
    BL_Azimuth.configNeutralDeadband(AZIMUTH_NEUTRAL_BAND);
    BL_Azimuth.setNeutralMode(NeutralMode.Brake);
    BL_Azimuth.configRemoteFeedbackFilter(BL_Position, 0);
    BL_Azimuth.setSelectedSensorPosition(BL_Position.getAbsolutePosition());
    BL_Azimuth.config_kP(0, AZIMUTH_kP);
    BL_Azimuth.config_kI(0, AZIMUTH_kI);
    BL_Azimuth.config_kD(0, AZIMUTH_kD);
    BL_Azimuth.config_IntegralZone(0, AZIMUTH_INTEGRAL_ZONE);

    BR_Azimuth.configFactoryDefault();
    BR_Azimuth.setInverted(TalonFXInvertType.CounterClockwise);
    BR_Azimuth.configNeutralDeadband(AZIMUTH_NEUTRAL_BAND);
    BR_Azimuth.setNeutralMode(NeutralMode.Brake);
    BR_Azimuth.configRemoteFeedbackFilter(BR_Position, 0);
    BR_Azimuth.setSelectedSensorPosition(BR_Position.getAbsolutePosition());
    BR_Azimuth.config_kP(0, AZIMUTH_kP);
    BR_Azimuth.config_kI(0, AZIMUTH_kI);
    BR_Azimuth.config_kD(0, AZIMUTH_kD);
    BR_Azimuth.config_IntegralZone(0, AZIMUTH_INTEGRAL_ZONE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // 'actual' read sensor positions of each module
    FL_Actual = (FL_Azimuth.getSelectedSensorPosition() / 4096) * 360;
    FR_Actual = (FR_Azimuth.getSelectedSensorPosition() / 4096) * 360;
    BL_Actual = (BL_Azimuth.getSelectedSensorPosition() / 4096) * 360;
    BR_Actual = (BR_Azimuth.getSelectedSensorPosition() / 4096) * 360;

    // dashboard data
    Telemetry.setValue("drivetrain/FL/Azimuth_Target", FL_Target);
    Telemetry.setValue("drivetrain/FR/Azimuth_Target", FR_Target);
    Telemetry.setValue("drivetrain/BL/Azimuth_Target", BL_Target);
    Telemetry.setValue("drivetrain/BR/Azimuth_Target", BR_Target);
    Telemetry.setValue("drivetrain/FL/Drive_Speed", FL_Speed);
    Telemetry.setValue("drivetrain/FR/Drive_Speed", FR_Speed);
    Telemetry.setValue("drivetrain/BL/Drive_Speed", BL_Speed);
    Telemetry.setValue("drivetrain/BR/Drive_Speed", BR_Speed);
    Telemetry.setValue("drivetrain/FL/Azimuth_Actual", FL_Actual);
    Telemetry.setValue("drivetrain/FR/Azimuth_Actual", FR_Actual);
    Telemetry.setValue("drivetrain/BL/Azimuth_Actual", BL_Actual);
    Telemetry.setValue("drivetrain/BR/Azimuth_Actual", BR_Actual);
    Telemetry.setValue("drivetrain/FL/Drive_Temp", FL_Drive.getTemperature());
    Telemetry.setValue("drivetrain/FR/Drive_Temp", FR_Drive.getTemperature());
    Telemetry.setValue("drivetrain/BL/Drive_Temp", BL_Drive.getTemperature());
    Telemetry.setValue("drivetrain/BR/Drive_Temp", BR_Drive.getTemperature());
    Telemetry.setValue("drivetrain/FL/Azimuth_Temp", FL_Azimuth.getTemperature());
    Telemetry.setValue("drivetrain/FR/Azimuth_Temp", FR_Azimuth.getTemperature());
    Telemetry.setValue("drivetrain/BL/Azimuth_Temp", BL_Azimuth.getTemperature());
    Telemetry.setValue("drivetrain/BR/Azimuth_Temp", BR_Azimuth.getTemperature());
    Telemetry.setValue("drivetrain/isRobotOriented", isRobotOriented);
    Telemetry.setValue("drivetrain/yaw", gyro.getYaw());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

    // dashboard data
    Telemetry.setValue("drivetrain/FL/Azimuth_Target", FL_Target);
    Telemetry.setValue("drivetrain/FR/Azimuth_Target", FR_Target);
    Telemetry.setValue("drivetrain/BL/Azimuth_Target", BL_Target);
    Telemetry.setValue("drivetrain/BR/Azimuth_Target", BR_Target);
    Telemetry.setValue("drivetrain/FL/Drive_Speed", FL_Speed);
    Telemetry.setValue("drivetrain/FR/Drive_Speed", FR_Speed);
    Telemetry.setValue("drivetrain/BL/Drive_Speed", BL_Speed);
    Telemetry.setValue("drivetrain/BR/Drive_Speed", BR_Speed);
    Telemetry.setValue("drivetrain/FL/Azimuth_Actual", FL_Actual);
    Telemetry.setValue("drivetrain/FR/Azimuth_Actual", FR_Actual);
    Telemetry.setValue("drivetrain/BL/Azimuth_Actual", BL_Actual);
    Telemetry.setValue("drivetrain/BR/Azimuth_Actual", BR_Actual);
    Telemetry.setValue("drivetrain/FL/Drive_Temp", FL_Drive.getTemperature());
    Telemetry.setValue("drivetrain/FR/Drive_Temp", FR_Drive.getTemperature());
    Telemetry.setValue("drivetrain/BL/Drive_Temp", BL_Drive.getTemperature());
    Telemetry.setValue("drivetrain/BR/Drive_Temp", BR_Drive.getTemperature());
    Telemetry.setValue("drivetrain/FL/Azimuth_Temp", FL_Azimuth.getTemperature());
    Telemetry.setValue("drivetrain/FR/Azimuth_Temp", FR_Azimuth.getTemperature());
    Telemetry.setValue("drivetrain/BL/Azimuth_Temp", BL_Azimuth.getTemperature());
    Telemetry.setValue("drivetrain/BR/Azimuth_Temp", BR_Azimuth.getTemperature());
    Telemetry.setValue("drivetrain/isRobotOriented", isRobotOriented);
    Telemetry.setValue("drivetrain/yaw", gyro.getYaw());
  }

  public void drive(double LX, double LY, double RX) {

    // apply deadzone of 0.1 to each axis
    if (Math.abs(LX) < 0.1)
      LX = 0.0;
    if (Math.abs(LY) < 0.1)
      LY = 0.0;
    if (Math.abs(RX) < 0.1)
      RX = 0.0;

    // vector addition of strafe component (LX & LY) and rotation component
    // (ROTATION_X * RX)
    FL_X = LX + (ROTATION_X * RX);
    FL_Y = LY + (ROTATION_Y * RX);

    // pythagorean theorum to find magnitude of resultant vector
    FL_Speed = Math.hypot(FL_X, FL_Y);
    // arctan to find angle of resulatant vector, then correct for quadrant
    FL_Target = (Math.toDegrees(Math.atan2(FL_Y, FL_X)) + (LY < 0 ? 360 : 0)) % 360;

    // vector addition of strafe component (LX & LY) and rotation component
    // (ROTATION_X * RX)
    FR_X = LX + (ROTATION_X * RX);
    FR_Y = LY - (ROTATION_Y * RX);

    // pythagorean theorum to find magnitude of resultant vector
    FR_Speed = Math.hypot(FR_X, FR_Y);
    // arctan to find angle of resulatant vector, then correct for quadrant
    FR_Target = (Math.toDegrees(Math.atan2(FR_Y, FR_X)) + (LY < 0 ? 360 : 0)) % 360;

    // vector addition of strafe component (LX & LY) and rotation component
    // (ROTATION_X * RX)
    BL_X = LX - (ROTATION_X * RX);
    BL_Y = LY + (ROTATION_Y * RX);

    // pythagorean theorum to find magnitude of resultant vector
    BL_Speed = Math.hypot(BL_X, BL_Y);
    // arctan to find angle of resulatant vector, then correct for quadrant
    BL_Target = (Math.toDegrees(Math.atan2(BL_Y, BL_X)) + (LY < 0 ? 360 : 0)) % 360;

    // vector addition of strafe component (LX & LY) and rotation component
    // (ROTATION_X * RX)
    BR_X = LX - (ROTATION_X * RX);
    BR_Y = LY - (ROTATION_Y * RX);

    // pythagorean theorum to find magnitude of resultant vector
    BR_Speed = Math.hypot(BR_X, BR_Y);
    // arctan to find angle of resulatant vector, then correct for quadrant
    BR_Target = (Math.toDegrees(Math.atan2(BR_Y, BR_X)) + (LY < 0 ? 360 : 0)) % 360;

    // if one or more wheels has a target speed of > 100%, put all wheel speeds over
    // the max so that the
    // fastest wheel goes full speed and the others preserve their ratios of speed
    // to acheive smooth movement
    _speedRegulator = Math.max(Math.max(FL_Speed, FR_Speed), Math.max(BL_Speed, BR_Speed));
    if (_speedRegulator > 1.0) {
      FL_Speed /= _speedRegulator;
      FR_Speed /= _speedRegulator;
      BL_Speed /= _speedRegulator;
      BR_Speed /= _speedRegulator;
    }

    // read yaw from NavX and apply offset
    robotYaw = gyro.getYaw() + GYRO_OFFSET;

    // field orientation
    FL_Target += isRobotOriented ? 0.0 : robotYaw;
    FR_Target += isRobotOriented ? 0.0 : robotYaw;
    BL_Target += isRobotOriented ? 0.0 : robotYaw;
    BR_Target += isRobotOriented ? 0.0 : robotYaw;

    // if joystick is idle, lock wheels to X formation to avoid pushing
    // TODO check if X-locking effective, if necesary; button toggle instead of default?
    if (LX == 0 && LY == 0 && RX == 0) {
      FL_Target = (Math.toDegrees(Math.atan2(+ROTATION_Y, -ROTATION_X))) % 360;
      FR_Target = (Math.toDegrees(Math.atan2(+ROTATION_Y, +ROTATION_X))) % 360;
      BL_Target = (Math.toDegrees(Math.atan2(-ROTATION_Y, -ROTATION_X))) % 360;
      BR_Target = (Math.toDegrees(Math.atan2(-ROTATION_Y, +ROTATION_X))) % 360;
    }

    // find the shortest path to an equivalent position to prevent unneccesary full
    // rotations
    _Path_1 = Math.abs(FL_Target - FL_Actual);
    _Path_2 = Math.abs((FL_Target + 360) - FL_Actual);
    _Path_3 = Math.abs((FL_Target - 360) - FL_Actual);
    if (Math.min(Math.min(_Path_1, _Path_2), _Path_3) == _Path_2)
      FL_Target += 360;
    if (Math.min(Math.min(_Path_1, _Path_2), _Path_3) == _Path_3)
      FL_Target -= 360;

    _Path_1 = Math.abs(FR_Target - FR_Actual);
    _Path_2 = Math.abs((FR_Target + 360) - FR_Actual);
    _Path_3 = Math.abs((FR_Target - 360) - FR_Actual);
    if (Math.min(Math.min(_Path_1, _Path_2), _Path_3) == _Path_2)
      FR_Target += 360;
    if (Math.min(Math.min(_Path_1, _Path_2), _Path_3) == _Path_3)
      FR_Target -= 360;

    _Path_1 = Math.abs(BL_Target - BL_Actual);
    _Path_2 = Math.abs((BL_Target + 360) - BL_Actual);
    _Path_3 = Math.abs((BL_Target - 360) - BL_Actual);
    if (Math.min(Math.min(_Path_1, _Path_2), _Path_3) == _Path_2)
      BL_Target += 360;
    if (Math.min(Math.min(_Path_1, _Path_2), _Path_3) == _Path_3)
      BL_Target -= 360;

    _Path_1 = Math.abs(BR_Target - BR_Actual);
    _Path_2 = Math.abs((BR_Target + 360) - BR_Actual);
    _Path_3 = Math.abs((BR_Target - 360) - BR_Actual);
    if (Math.min(Math.min(_Path_1, _Path_2), _Path_3) == _Path_2)
      BR_Target += 360;
    if (Math.min(Math.min(_Path_1, _Path_2), _Path_3) == _Path_3)
      BR_Target -= 360;

    // correct the target positions so that they are close to the current position
    // then convert to sensor units and pass target positions to motor controllers
    FL_Azimuth.set(ControlMode.Position, ((FL_Target + (FL_Actual - (FL_Actual % 360) / 360) * 4096)));
    FR_Azimuth.set(ControlMode.Position, ((FR_Target + (FR_Actual - (FR_Actual % 360) / 360) * 4096)));
    BL_Azimuth.set(ControlMode.Position, ((BL_Target + (BL_Actual - (BL_Actual % 360) / 360) * 4096)));
    BR_Azimuth.set(ControlMode.Position, ((BR_Target + (BR_Actual - (BR_Actual % 360) / 360) * 4096)));

    // pass wheel speeds to motor controllers
    FL_Drive.set(ControlMode.PercentOutput, FL_Speed);
    FR_Drive.set(ControlMode.PercentOutput, FR_Speed);
    BL_Drive.set(ControlMode.PercentOutput, BL_Speed);
    BR_Drive.set(ControlMode.PercentOutput, BR_Speed);
  }

  /** Sets the gyroscope's current heading to 0 */
  public void zeroGyro() {
    gyro.zeroYaw();
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
