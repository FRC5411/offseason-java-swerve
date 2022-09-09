// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import frc.lib.Telemetry;

public class Swerve extends SubsystemBase {
  private Joystick driver;
  private AHRS gyro;

  // controller axis values
  private double LX = 0.0;
  private double LY = 0.0;
  private double RX = 0.0;

  // field position
  private Double[] position_x = new Double[4];
  private Double[] position_y = new Double[4];

  private double FL_Distance = 0;
  private double FR_Distance = 0;
  private double BL_Distance = 0;
  private double BR_Distance = 0;

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
  private double Path_1 = 0.0;
  private double Path_2 = 0.0;
  private double Path_3 = 0.0;

  // swerve wheel speed ratio preservation while capping speeds at 100%
  private double speedRegulator = 1.0;

  // the read yaw value from the NavX
  private double robotYaw = 0.0;
  // the rotation offset of the NavX to the robot (degrees)
  private final double gyroOffset = 0.0;

  // robot oriented / field oriented swerve drive toggle
  private boolean isRobotOriented  = true;
  private boolean orientationState = false;

  // length = front to back (inches)
  public static final double ROBOT_LENGTH = 31.0;
  // width = side to side (inches)
  public static final double ROBOT_WIDTH = 28.0;

  // constants for calculating rotation vector
  private final double ROTATION_Y = Math.sin(Math.atan2(ROBOT_LENGTH, ROBOT_WIDTH));
  private final double ROTATION_X = Math.cos(Math.atan2(ROBOT_LENGTH, ROBOT_WIDTH));

  /** Creates a new ExampleSubsystem. */
  public Swerve ( Joystick usb0, AHRS NavX ) {

    driver = usb0;
    gyro = NavX;

    // config drive motors
    FL_Drive.configFactoryDefault();
    FL_Drive.setInverted(TalonFXInvertType.CounterClockwise);
    FL_Drive.setNeutralMode(NeutralMode.Brake);
    FL_Drive.configNeutralDeadband(0.001); // TODO tune if needed

    FR_Drive.configFactoryDefault();
    FR_Drive.setInverted(TalonFXInvertType.CounterClockwise);
    FR_Drive.setNeutralMode(NeutralMode.Brake);
    FR_Drive.configNeutralDeadband(0.001); // TODO tune if needed

    BL_Drive.configFactoryDefault();
    BL_Drive.setInverted(TalonFXInvertType.CounterClockwise);
    BL_Drive.setNeutralMode(NeutralMode.Brake);
    BL_Drive.configNeutralDeadband(0.001); // TODO tune if needed

    BR_Drive.configFactoryDefault();
    BR_Drive.setInverted(TalonFXInvertType.CounterClockwise);
    BR_Drive.setNeutralMode(NeutralMode.Brake);
    BR_Drive.configNeutralDeadband(0.001); // TODO tune if needed

    // config azimuth (steering) motors
    // TODO zeroing with cancoders
    FL_Azimuth.configFactoryDefault();
    FL_Azimuth.setInverted(TalonFXInvertType.CounterClockwise);
    FL_Azimuth.configNeutralDeadband(0.001); // TODO tune if needed
    FL_Azimuth.setNeutralMode(NeutralMode.Brake);
    FL_Azimuth.configRemoteFeedbackFilter(FL_Position, 0);

    FR_Azimuth.configFactoryDefault();
    FR_Azimuth.setInverted(TalonFXInvertType.CounterClockwise);
    FR_Azimuth.configNeutralDeadband(0.001); // TODO tune if needed
    FR_Azimuth.setNeutralMode(NeutralMode.Brake);
    FR_Azimuth.configRemoteFeedbackFilter(FR_Position, 0);

    BL_Azimuth.configFactoryDefault();
    BL_Azimuth.setInverted(TalonFXInvertType.CounterClockwise);
    BL_Azimuth.configNeutralDeadband(0.001); // TODO tune if needed
    BL_Azimuth.setNeutralMode(NeutralMode.Brake);
    BL_Azimuth.configRemoteFeedbackFilter(BL_Position, 0);

    BR_Azimuth.configFactoryDefault();
    BR_Azimuth.setInverted(TalonFXInvertType.CounterClockwise);
    BR_Azimuth.configNeutralDeadband(0.001); // TODO tune if needed
    BR_Azimuth.setNeutralMode(NeutralMode.Brake);
    BR_Azimuth.configRemoteFeedbackFilter(BR_Position, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // 'actual' read sensor positions of each module
    FL_Actual = (FL_Azimuth.getSelectedSensorPosition()/4096)*360;
    FR_Actual = (FR_Azimuth.getSelectedSensorPosition()/4096)*360;
    BL_Actual = (BL_Azimuth.getSelectedSensorPosition()/4096)*360;
    BR_Actual = (BR_Azimuth.getSelectedSensorPosition()/4096)*360;

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
    Telemetry.setValue("drivetrain/FL/Drive_Distance", FL_Distance);
    Telemetry.setValue("drivetrain/FR/Drive_Distance", FR_Distance);
    Telemetry.setValue("drivetrain/BL/Drive_Distance", BL_Distance);
    Telemetry.setValue("drivetrain/BR/Drive_Distance", BR_Distance);
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
  }

  public void drive () {
    // fetch joystick axis values
    LX = driver.getRawAxis(0); // left x axis (strafe)
    LY = -driver.getRawAxis(1); // left y axis (strafe)
    RX = driver.getRawAxis(4); // right x axis (rotation)

    // apply deadzone of 0.1 to each axis
    if ( Math.abs(LX) < 0.1 ) LX = 0.0;
    if ( Math.abs(LY) < 0.1 ) LY = 0.0;
    if ( Math.abs(RX) < 0.1 ) RX = 0.0;

    // vector addition of strafe component (LX & LY) and rotation component (ROTATION_X * RX)
    FL_X = LX + (ROTATION_X * RX);
    FL_Y = LY + (ROTATION_Y * RX);

    // pythagorean theorum to find magnitude of resultant vector
    FL_Speed  = Math.sqrt( Math.pow(FL_X, 2) + Math.pow(FL_Y, 2) );
    // arctan to find angle of resulatant vector, then correct for quadrant
    FL_Target = (Math.toDegrees(Math.atan2(FL_Y, FL_X)) + (LY < 0 ? 360 : 0)) % 360;

    // vector addition of strafe component (LX & LY) and rotation component (ROTATION_X * RX)
    FR_X = LX + (ROTATION_X * RX);
    FR_Y = LY - (ROTATION_Y * RX);

    // pythagorean theorum to find magnitude of resultant vector
    FR_Speed  = Math.sqrt( Math.pow(FR_X, 2) + Math.pow(FR_Y, 2) );
    // arctan to find angle of resulatant vector, then correct for quadrant
    FR_Target = (Math.toDegrees(Math.atan2(FR_Y, FR_X)) + (LY < 0 ? 360 : 0)) % 360;

    // vector addition of strafe component (LX & LY) and rotation component (ROTATION_X * RX)
    BL_X = LX - (ROTATION_X * RX);
    BL_Y = LY + (ROTATION_Y * RX);

    // pythagorean theorum to find magnitude of resultant vector
    BL_Speed  = Math.sqrt( Math.pow(BL_X, 2) + Math.pow(BL_Y, 2) );
    // arctan to find angle of resulatant vector, then correct for quadrant
    BL_Target = (Math.toDegrees(Math.atan2(BL_Y, BL_X)) + (LY < 0 ? 360 : 0)) % 360;

    // vector addition of strafe component (LX & LY) and rotation component (ROTATION_X * RX)
    BR_X = LX - (ROTATION_X * RX);
    BR_Y = LY - (ROTATION_Y * RX);

    // pythagorean theorum to find magnitude of resultant vector
    BR_Speed  = Math.sqrt( Math.pow(BR_X, 2) + Math.pow(BR_Y, 2) );
    // arctan to find angle of resulatant vector, then correct for quadrant
    BR_Target = (Math.toDegrees(Math.atan2(BR_Y, BR_X)) + (LY < 0 ? 360 : 0)) % 360;

    // if one or more wheels has a target speed of > 100%, put all wheel speeds over the max so that the
    // fastest wheel goes full speed and the others preserve their ratios of speed to acheive smooth movement
    speedRegulator = Math.max( Math.max( FL_Speed, FR_Speed ), Math.max(BL_Speed, BR_Speed) );
    if ( speedRegulator > 1.0 ) {
      FL_Speed /= speedRegulator;
      FR_Speed /= speedRegulator;
      BL_Speed /= speedRegulator;
      BR_Speed /= speedRegulator;
    }

    // robot orient / field orient toggle on A button pressed
    if ( !orientationState && driver.getRawButton(1) ) isRobotOriented = !isRobotOriented;
    orientationState = driver.getRawButton(1);

    // zero NavX on B button pressed
    if ( driver.getRawButton(2) ) gyro.zeroYaw();

    // read yaw from NavX and apply offset
    robotYaw = gyro.getYaw() + gyroOffset;

    // field orientation
    FL_Target += isRobotOriented ? 0.0 : robotYaw;
    FR_Target += isRobotOriented ? 0.0 : robotYaw;
    BL_Target += isRobotOriented ? 0.0 : robotYaw;
    BR_Target += isRobotOriented ? 0.0 : robotYaw;

    // if joystick is idle, lock wheels to X formation to avoid pushing
    // TODO check if effective, if necesary; button toggle instead of default?
    if ( LX == 0 && LY == 0 && RX == 0 ) {
      FL_Target = (Math.toDegrees(Math.atan2(+ROTATION_Y, -ROTATION_X))) % 360;
      FR_Target = (Math.toDegrees(Math.atan2(+ROTATION_Y, +ROTATION_X))) % 360;
      BL_Target = (Math.toDegrees(Math.atan2(-ROTATION_Y, -ROTATION_X))) % 360;
      BR_Target = (Math.toDegrees(Math.atan2(-ROTATION_Y, +ROTATION_X))) % 360;
    }

    // find the shortest path to an equivalent position to prevent unneccesary full rotations
    Path_1 = Math.abs(FL_Target - FL_Actual);
    Path_2 = Math.abs((FL_Target + 360) - FL_Actual);
    Path_3 = Math.abs((FL_Target - 360) - FL_Actual);
    if ( Math.min(Math.min( Path_1, Path_2 ), Path_3) == Path_2 ) FL_Target += 360;
    if ( Math.min(Math.min( Path_1, Path_2 ), Path_3) == Path_3 ) FL_Target -= 360;

    Path_1 = Math.abs(FR_Target - FR_Actual);
    Path_2 = Math.abs((FR_Target + 360) - FR_Actual);
    Path_3 = Math.abs((FR_Target - 360) - FR_Actual);
    if ( Math.min(Math.min( Path_1, Path_2 ), Path_3) == Path_2 ) FR_Target += 360;
    if ( Math.min(Math.min( Path_1, Path_2 ), Path_3) == Path_3 ) FR_Target -= 360;

    Path_1 = Math.abs(BL_Target - BL_Actual);
    Path_2 = Math.abs((BL_Target + 360) - BL_Actual);
    Path_3 = Math.abs((BL_Target - 360) -BL_Actual);
    if ( Math.min(Math.min( Path_1, Path_2 ), Path_3) == Path_2 ) BL_Target += 360;
    if ( Math.min(Math.min( Path_1, Path_2 ), Path_3) == Path_3 ) BL_Target -= 360;

    Path_1 = Math.abs(BR_Target - BR_Actual);
    Path_2 = Math.abs((BR_Target + 360) - BR_Actual);
    Path_3 = Math.abs((BR_Target - 360) - BR_Actual);
    if ( Math.min(Math.min( Path_1, Path_2 ), Path_3) == Path_2 ) BR_Target += 360;
    if ( Math.min(Math.min( Path_1, Path_2 ), Path_3) == Path_3 ) BR_Target -= 360;

    // correct the target positions so that they are close to the current position
    // then convert to sensor units and pass target positions to motor controllers
    FL_Azimuth.set(ControlMode.Position, ((FL_Target + (FL_Actual - (FL_Actual % 360)/360)*4096)));
    FR_Azimuth.set(ControlMode.Position, ((FR_Target + (FR_Actual - (FR_Actual % 360)/360)*4096)));
    BL_Azimuth.set(ControlMode.Position, ((BL_Target + (BL_Actual - (BL_Actual % 360)/360)*4096)));
    BR_Azimuth.set(ControlMode.Position, ((BR_Target + (BR_Actual - (BR_Actual % 360)/360)*4096)));

    // pass wheel speeds to motor controllers
    FL_Drive.set(ControlMode.PercentOutput, FL_Speed);
    FR_Drive.set(ControlMode.PercentOutput, FR_Speed);
    BL_Drive.set(ControlMode.PercentOutput, BL_Speed);
    BR_Drive.set(ControlMode.PercentOutput, BR_Speed);
  }
}
