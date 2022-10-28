// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;
import java.util.function.Consumer;
import java.util.function.Supplier;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.server.PathPlannerServer;

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

  // robot oriented / field oriented swerve drive toggle
  private boolean isRobotOriented = true;
  // length = front to back (meters)
  public static final double ROBOT_LENGTH_METERS = 0.7874;
  // width = side to side (meters)
  public static final double ROBOT_WIDTH_METERS = 0.7112;
  // wheel diameter (meters)
  public static final double WHEEL_DIAMETER_METERS = 0.1016;
  // drive gear ratio
  public static final double DRIVE_GEAR_RATIO = 6.75;

  // constants for calculating rotation vector
  private static final double ROTATION_Y = Math.sin(Math.atan2(ROBOT_LENGTH_METERS, ROBOT_WIDTH_METERS));
  private static final double ROTATION_X = Math.cos(Math.atan2(ROBOT_LENGTH_METERS, ROBOT_WIDTH_METERS));
  
  private static final StatorCurrentLimitConfiguration DRIVE_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(); // TODO tune drive motor stator current limit

  // encoder offsets (degrees)
  private static final double FL_ECODER_OFFSET = -313.682;
  private static final double FR_ECODER_OFFSET = -166.553;
  private static final double BL_ECODER_OFFSET = -246.006;
  private static final double BR_ECODER_OFFSET = -204.258;

  // pid values
  private static final double AZIMUTH_kP = 0.4;
  private static final double AZIMUTH_kD = 12;

  // calculated via JVN calculator
  private static final double DRIVE_kP = 0.044057;
  private static final double DRIVE_kF = 0.028998;

  // TODO find actual max speeds
  /** maximum strafe speed (meters per second) */
  private static final double MAX_LINEAR_SPEED = 5.4;
  /** maximum rotation speed (radians per second) */
  private static final double MAX_ROTATION_SPEED = Math.PI*2;

  /** WPILib swerve kinematics */
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    new Translation2d(ROBOT_LENGTH_METERS/2, ROBOT_WIDTH_METERS/2), 
    new Translation2d(ROBOT_LENGTH_METERS/2, -ROBOT_WIDTH_METERS/2), 
    new Translation2d(-ROBOT_LENGTH_METERS/2, ROBOT_WIDTH_METERS/2), 
    new Translation2d(-ROBOT_LENGTH_METERS/2, -ROBOT_WIDTH_METERS/2)
  );

  private ChassisSpeeds forwardKinematics = new ChassisSpeeds();

  private ChassisSpeeds kinematicCommand = new ChassisSpeeds();

  private SwerveModuleState[] modules = new SwerveModuleState[3];

  private short[] accelerometerData = new short[3];

  private SwerveDrivePoseEstimator odometryOfficial = new SwerveDrivePoseEstimator(
    new Rotation2d(), 
    new Pose2d(), 
    kinematics, 
    new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), 
    new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.02), 
    new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)
  );
  private Pose2d _robotPose = new Pose2d();

  private Consumer<SwerveModuleState[]> pathFollower = modules -> {
    driveFromModuleStates( modules);
  };

  private Supplier<Pose2d> pathState = () -> {
    return odometryOfficial.getEstimatedPosition();
  };

  /** Creates a new ExampleSubsystem. */
  public Swerve(Pigeon2 pigeon) {

    gyro = pigeon;

    // config drive motors
    configDrive(FL_Drive);
    configDrive(FR_Drive);
    configDrive(BL_Drive);
    configDrive(BR_Drive);

    // config CANcoders
    configPosition(FL_Position, FL_ECODER_OFFSET);
    configPosition(FR_Position, FR_ECODER_OFFSET);
    configPosition(BL_Position, BL_ECODER_OFFSET);
    configPosition(BR_Position, BR_ECODER_OFFSET);

    // config azimuth (steering) motors
    configAzimuth(FL_Azimuth, FL_Position);
    configAzimuth(FR_Azimuth, FR_Position);
    configAzimuth(BL_Azimuth, BL_Position);
    configAzimuth(BR_Azimuth, BR_Position);

    PathPlannerServer.startServer(6969); // 5811 = port number. adjust this according to your needs
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
    FL_Actual_Speed = 2.0*(((FL_Drive.getSelectedSensorVelocity() / 4096) * 10) / DRIVE_GEAR_RATIO) * Math.PI * WHEEL_DIAMETER_METERS;
    FR_Actual_Speed = 2.0*(((FR_Drive.getSelectedSensorVelocity() / 4096) * 10) / DRIVE_GEAR_RATIO) * Math.PI * WHEEL_DIAMETER_METERS;
    BL_Actual_Speed = 2.0*(((BL_Drive.getSelectedSensorVelocity() / 4096) * 10) / DRIVE_GEAR_RATIO) * Math.PI * WHEEL_DIAMETER_METERS;
    BR_Actual_Speed = 2.0*(((BR_Drive.getSelectedSensorVelocity() / 4096) * 10) / DRIVE_GEAR_RATIO) * Math.PI * WHEEL_DIAMETER_METERS;

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
    Telemetry.setValue("drivetrain/Modules/FL/Azimuth/Output_Voltage", FL_Azimuth.getMotorOutputVoltage());
    Telemetry.setValue("drivetrain/Modules/FR/Azimuth/Output_Voltage", FR_Azimuth.getMotorOutputVoltage());
    Telemetry.setValue("drivetrain/Modules/BL/Azimuth/Output_Voltage", BL_Azimuth.getMotorOutputVoltage());
    Telemetry.setValue("drivetrain/Modules/BR/Azimuth/Output_Voltage", BR_Azimuth.getMotorOutputVoltage());
    Telemetry.setValue("drivetrain/Modules/FL/Drive/Output_Voltage", FL_Drive.getMotorOutputVoltage());
    Telemetry.setValue("drivetrain/Modules/FR/Drive/Output_Voltage", FR_Drive.getMotorOutputVoltage());
    Telemetry.setValue("drivetrain/Modules/BL/Drive/Output_Voltage", BL_Drive.getMotorOutputVoltage());
    Telemetry.setValue("drivetrain/Modules/BR/Drive/Output_Voltage", BR_Drive.getMotorOutputVoltage());
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

    forwardKinematics = kinematics.toChassisSpeeds(new SwerveModuleState(FL_Actual_Speed, new Rotation2d(Math.toRadians(FL_Actual_Position))), new SwerveModuleState(FR_Actual_Speed, new Rotation2d(Math.toRadians(FR_Actual_Position))), new SwerveModuleState(BL_Actual_Speed, new Rotation2d(Math.toRadians(BL_Actual_Position))), new SwerveModuleState(BR_Actual_Speed, new Rotation2d(Math.toRadians(BR_Actual_Position))) );

    Telemetry.setValue("drivetrain/kinematics/official/robot/forward", forwardKinematics.vxMetersPerSecond);
    Telemetry.setValue("drivetrain/kinematics/official/robot/rightward", -forwardKinematics.vyMetersPerSecond);
    Telemetry.setValue("drivetrain/kinematics/official/clockwise_speed", Math.toDegrees(forwardKinematics.omegaRadiansPerSecond));
    Telemetry.setValue("drivetrain/kinematics/official/field/DS_away", ( forwardKinematics.vxMetersPerSecond * Math.cos(Math.toRadians(gyro.getYaw())) - forwardKinematics.vyMetersPerSecond * Math.sin(Math.toRadians(gyro.getYaw()))));
    Telemetry.setValue("drivetrain/kinematics/official/field/DS_right", ( -forwardKinematics.vyMetersPerSecond * Math.cos(Math.toRadians(gyro.getYaw())) - forwardKinematics.vxMetersPerSecond * Math.sin(Math.toRadians(gyro.getYaw()))));

    _robotPose = odometryOfficial.update(new Rotation2d(Math.toRadians(gyro.getYaw())), new SwerveModuleState(FL_Actual_Speed, new Rotation2d(Math.toRadians(FL_Actual_Position))), new SwerveModuleState(FR_Actual_Speed, new Rotation2d(Math.toRadians(FR_Actual_Position))), new SwerveModuleState(BL_Actual_Speed, new Rotation2d(Math.toRadians(BL_Actual_Position))), new SwerveModuleState(BR_Actual_Speed, new Rotation2d(Math.toRadians(BR_Actual_Position))) );

    Telemetry.setValue("drivetrain/odometry/official/field/DS_away", -_robotPose.getX());
    Telemetry.setValue("drivetrain/odometry/official/field/DS_right", _robotPose.getY());

    gyro.getBiasedAccelerometer(accelerometerData);
    Telemetry.setValue("drivetrain/kinematics/gyro/robot/forward_acceleration", (double) accelerometerData[1]);
    Telemetry.setValue("drivetrain/kinematics/gyro/robot/rightward_acceleration", (double) accelerometerData[0]);
  }

  @Override
  public void simulationPeriodic() {}

  public void joystickDrive(double LX, double LY, double RX) {

    // WPILib swerve command
    kinematicCommand = new ChassisSpeeds(LY * MAX_LINEAR_SPEED, -LX * MAX_LINEAR_SPEED, -RX * MAX_ROTATION_SPEED);
    if ( !isRobotOriented ) kinematicCommand = ChassisSpeeds.fromFieldRelativeSpeeds(LY * MAX_LINEAR_SPEED, -LX * MAX_LINEAR_SPEED, -RX * MAX_ROTATION_SPEED, Rotation2d.fromDegrees(gyro.getYaw()));
    
    modules = kinematics.toSwerveModuleStates(kinematicCommand);

    // if joystick is idle, lock wheels to X formation to avoid pushing
    if (LX == 0 && LY == 0 && RX == 0) {
      modules[0].angle = new Rotation2d((Math.atan2( ROTATION_Y,  ROTATION_X)) % 360);
      modules[1].angle = new Rotation2d((Math.atan2( ROTATION_Y, -ROTATION_X)) % 360);
      modules[2].angle = new Rotation2d((Math.atan2(-ROTATION_Y,  ROTATION_X)) % 360);
      modules[3].angle = new Rotation2d((Math.atan2(-ROTATION_Y, -ROTATION_X)) % 360);
    }

    driveFromModuleStates(modules);
  }

  public void driveFromModuleStates ( SwerveModuleState[] modules ) {
    SwerveDriveKinematics.desaturateWheelSpeeds(modules, MAX_LINEAR_SPEED);

    FL_Target = modules[0].angle.getDegrees() % 360;
    FR_Target = modules[1].angle.getDegrees() % 360;
    BL_Target = modules[2].angle.getDegrees() % 360;
    BR_Target = modules[3].angle.getDegrees() % 360;
    FL_Speed = modules[0].speedMetersPerSecond;
    FR_Speed = modules[1].speedMetersPerSecond;
    BL_Speed = modules[2].speedMetersPerSecond;
    BR_Speed = modules[3].speedMetersPerSecond;

    // find the shortest path to an equivalent position to prevent unneccesary full rotations
    FL_Target = optimizeAzimuthPath(FL_Target, FL_Actual_Position);
    FR_Target = optimizeAzimuthPath(FR_Target, FR_Actual_Position);
    BL_Target = optimizeAzimuthPath(BL_Target, BL_Actual_Position);
    BR_Target = optimizeAzimuthPath(BR_Target, BR_Actual_Position);

    // correct the target positions so that they are close to the current position
    // then convert to sensor units and pass target positions to motor controllers
    FL_Azimuth.set(ControlMode.Position, ((FL_Target + (FL_Actual_Position - (FL_Actual_Position % 360))) / 360) * 4096);
    FR_Azimuth.set(ControlMode.Position, ((FR_Target + (FR_Actual_Position - (FR_Actual_Position % 360))) / 360) * 4096);
    BL_Azimuth.set(ControlMode.Position, ((BL_Target + (BL_Actual_Position - (BL_Actual_Position % 360))) / 360) * 4096);
    BR_Azimuth.set(ControlMode.Position, ((BR_Target + (BR_Actual_Position - (BR_Actual_Position % 360))) / 360) * 4096);

    // pass wheel speeds to motor controllers
    FL_Drive.set(ControlMode.Velocity, (FL_Speed*DRIVE_GEAR_RATIO/(Math.PI * WHEEL_DIAMETER_METERS)*4096)/10);
    FR_Drive.set(ControlMode.Velocity, (FR_Speed*DRIVE_GEAR_RATIO/(Math.PI * WHEEL_DIAMETER_METERS)*4096)/10);
    BL_Drive.set(ControlMode.Velocity, (BL_Speed*DRIVE_GEAR_RATIO/(Math.PI * WHEEL_DIAMETER_METERS)*4096)/10);
    BR_Drive.set(ControlMode.Velocity, (BR_Speed*DRIVE_GEAR_RATIO/(Math.PI * WHEEL_DIAMETER_METERS)*4096)/10);
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

  /** Sets the robot's orientation to robot (true) or field (false) 
   * @param _isRobotOriented - false if the robot should move with the gyro
  */
  public void setRobotOriented(boolean _isRobotOriented) {
    isRobotOriented = _isRobotOriented;
  }

  /** runs the configuration methods to apply the config variables 
   * @param motor - the device to configure
  */
  private void configDrive (TalonFX motor) {
    motor.configFactoryDefault();
    motor.setInverted(TalonFXInvertType.CounterClockwise);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configStatorCurrentLimit(DRIVE_CURRENT_LIMIT);
    motor.config_kP(0, DRIVE_kP);
    motor.config_kF(0, DRIVE_kF);
  }

  /** runs the configuration methods to apply the config variables 
   * @param motor - the device to configure
   * @param position - the CANCoder associated with the module
  */
  private void configAzimuth (TalonFX motor, CANCoder position) {
    motor.configFactoryDefault();
    motor.setInverted(TalonFXInvertType.CounterClockwise);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configRemoteFeedbackFilter(position, 0);
    motor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    motor.setSelectedSensorPosition(FL_Position.getAbsolutePosition());
    motor.config_kP(0, AZIMUTH_kP);
    motor.config_kD(0, AZIMUTH_kD);
  }
  
  /** runs the configuration methods to apply the config variables 
   * @param encoder - the device to configure
   * @param offset - the measured constant offset in degrees
  */
  private void configPosition (CANCoder encoder, double offset) {
    encoder.configFactoryDefault();
    encoder.configMagnetOffset(offset);
    encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    encoder.setPositionToAbsolute();
  }

  /** finds the closest equivalent position to the target 
   * @param target - the target direction of the module (deg)
   * @param actual - the current direction of the module (deg)
   * @return the corrected target position of the motor
  */
  private double optimizeAzimuthPath (double target, double actual) {
    if (Math.min(Math.min(Math.abs(target - FL_Actual_Position), Math.abs((target + 360) - actual)), Math.abs((target - 360) - actual)) == Math.abs((target + 360) - actual))
      target += 360;
    if (Math.min(Math.min(Math.abs(target - FL_Actual_Position), Math.abs((target + 360) - actual)), Math.abs((target - 360) - actual)) == Math.abs((target - 360) - actual))
      target -= 360;
    return target;
  }

  // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
public Command followTrajectoryCommand(PathPlannerTrajectory traj) {

  odometryOfficial.resetPosition(traj.getInitialHolonomicPose(), new Rotation2d(Math.toRadians(gyro.getYaw())));

  // This is just an example event map. It would be better to have a constant, global event map
  // in your code that will be used by all path following commands.
  HashMap<String, Command> eventMap = new HashMap<>();
  eventMap.put("marker1", new PrintCommand("Passed marker 1"));

  return new PPSwerveControllerCommand(
          traj, 
          pathState, // Pose supplier
          this.kinematics, // SwerveDriveKinematics
          // TODO tune pathplanner position PID
          new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
          new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          pathFollower, // Module states consumer
          eventMap, // This argument is optional if you don't use event markers
          this // Requires this drive subsystem
      );
  }
}