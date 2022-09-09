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

public class Swerve extends SubsystemBase {

    private Joystick driver;
    private AHRS NavX;
  
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
    private final double NavXOffset = 0.0;
  
    // robot oriented / field oriented swerve drive toggle
    private boolean isRobotOriented  = true;
    private boolean orientationState = false;
  
    // length = front to back
    private final double ROBOT_LENGTH = 31.0;
    // width = side to side
    private final double ROBOT_WIDTH = 28.0;
  
    // constants for calculating rotation vector
    private final double ROTATION_Y = Math.sin(Math.atan2(ROBOT_LENGTH, ROBOT_WIDTH));
    private final double ROTATION_X = Math.cos(Math.atan2(ROBOT_LENGTH, ROBOT_WIDTH));

  /** Creates a new ExampleSubsystem. */
  public Swerve () {

    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
