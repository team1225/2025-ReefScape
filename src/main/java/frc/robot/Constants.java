// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static class AprilTags {

		// facing the field elements (or from the driver station for the stages)
		public static final int RIGHT_BLUE_SOURCE = 1; 
		public static final int LEFT_BLUE_SOURCE = 2;
		public static final int RIGHT_RED_SPEAKER = 3;
		public static final int MIDDLE_RED_SPEAKER = 4; // target
		public static final int RED_AMP = 5; // target
		public static final int BLUE_AMP = 6; // target
		public static final int MIDDLE_BLUE_SPEAKER = 7; // target
		public static final int LEFT_BLUE_SPEAKER = 8;
		public static final int RIGHT_RED_SOURCE = 9;
		public static final int LEFT_RED_SOURCE = 10;
		public static final int STAGE_LEFT_RED_TRAP = 11;
		public static final int STAGE_RIGHT_RED_TRAP = 12;
		public static final int CENTER_STAGE_RED_TRAP = 13;
		public static final int CENTER_STAGE_BLUE_TRAP = 14;
		public static final int STAGE_LEFT_BLUE_TRAP = 15;
		public static final int STAGE_RIGHT_BLUE_TRAP = 16;
	
		public AprilTags() {
		}
	}

	public static class CameraConstants {
		public static final double cameraHeightFromGround = Units.inchesToMeters(18);
		public static final double cameraDistanceFromFront = Units.inchesToMeters(13.5 - 15);
		public static final double cameraDistanceFromRight = Units.inchesToMeters(13.5 - 13.5);
	}

	public static final class DrivetrainConstants {
		// Driving Parameters - Note that these are not the maximum capable speeds of
		// the robot, rather the allowed maximum speeds
		public static final double MAX_SPEED_METERS_PER_SECOND = 4.0; //4.42; //4.8;
		public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * Math.PI; // radians per second

		public static final double DIRECTION_SLEW_RATE = 1.2; // radians per second
		public static final double MAGNITUDE_SLEW_RATE = 1.8; // 2.0; //1.8; // percent per second (1 = 100%)
		public static final double ROTATIONAL_SLEW_RATE = 2.0; // 20.0; //2.0; // percent per second (1 = 100%)

		// Chassis configuration
		public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(18.75);
		
		// Distance between centers of right and left wheels on robot
		public static final double WHEEL_BASE_METERS = Units.inchesToMeters(18.75);
		
		// Distance between front and back wheels on robot
		public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
				new Translation2d(WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
				new Translation2d(WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2),
				new Translation2d(-WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
				new Translation2d(-WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2));

		public static final boolean kGyroReversed = false;
	}

	public static final class SwerveModuleConstants {
		// The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
		// This changes the drive speed of the module (a pinion gear with more teeth will result in a
		// robot that drives faster).
		public static final int kDrivingMotorPinionTeeth = 14;

		// Invert the turning encoder, since the output shaft rotates in the opposite direction of
		// the steering motor in the MAXSwerve Module.
		public static final boolean kTurningEncoderInverted = false;

		// Calculations required for driving motor conversion factors and feed forward
		public static final double DRIVING_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.FREE_SPEED_RPM / 60;
		public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
		public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
		// 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
		public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 17 * 50) / (kDrivingMotorPinionTeeth * 15 * 27);
		public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS)	/ DRIVING_MOTOR_REDUCTION;

		public static final double DRIVING_ENCODER_POSITION_FACTOR_METERS_PER_ROTATION = (WHEEL_DIAMETER_METERS * Math.PI) / DRIVING_MOTOR_REDUCTION; // meters, per rotation
		public static final double DRIVING_ENCODER_VELOCITY_FACTOR_METERS_PER_SECOND_PER_RPM = ((WHEEL_DIAMETER_METERS * Math.PI) / DRIVING_MOTOR_REDUCTION) / 60.0; // meters per second, per RPM

		public static final double TURNING_ENCODER_POSITION_FACTOR_RADIANS_PER_ROTATION = (2 * Math.PI); // radians, per rotation
		public static final double TURNING_ENCODER_VELOCITY_FACTOR_RADIANS_PER_SECOND_PER_RPM = (2 * Math.PI) / 60.0; // radians per second, per RPM

		public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT_RADIANS = -(2 * Math.PI); // radians
		public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT_RADIANS = (2 * Math.PI); // radians

		public static final double DRIVING_P = 0.04;
		public static final double DRIVING_I = 0;
		public static final double DRIVING_D = 0;
		public static final double DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
		public static final double DRIVING_MIN_OUTPUT_NORMALIZED = -1;
		public static final double DRIVING_MAX_OUTPUT_NORMALIZED = 1;

		public static final double TURNING_P = .9; // 1.0 might be a bit too much - reduce a bit if needed
		public static final double TURNING_I = 0;
		public static final double TURNING_D = 0;
		public static final double TURNING_FF = 0;
		public static final double TURNING_MIN_OUTPUT_NORMALIZED = -.25;
		public static final double TURNING_MAX_OUTPUT_NORMALIZED = .25;

		public static final IdleMode DRIVING_MOTOR_IDLE_MODE = IdleMode.kBrake;
		public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;

		public static final int DRIVING_MOTOR_CURRENT_LIMIT_AMPS = 80; //50; // amps
		public static final int TURNING_MOTOR_CURRENT_LIMIT_AMPS = 20; // amps
	}

	public static final class AutoConstants {
		public static final double MAX_SPEED_METERS_PER_SECOND = 3.0; //4.42; //3.0;
		public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
		public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
		public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = Math.PI;

		public static final double HIGH_SPEED_METERS_PER_SECOND = 5.0;

		public static final double REDUCED_SPEED_METERS_PER_SECOND = 2.0; //4.42; //3.0;

		public static final double X_CONTROLLER_P = 1;
		public static final double Y_CONTROLLER_P = 1;
		public static final double THETA_CONTROLLER_P = 1;

		// Constraint for the motion profiled robot angle controller
		public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
			MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);
	}

	public static final class NeoMotorConstants {
		public static final double FREE_SPEED_RPM = 5676;
	}

	public static final class PivotArmConstants {
		public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;
		public static final int TURNING_MOTOR_CURRENT_LIMIT_AMPS = 80; // amps

		public static final double TURNING_ENCODER_POSITION_FACTOR_RADIANS_PER_ROTATION = (2 * Math.PI); // radians, per rotation
		public static final double TURNING_ENCODER_VELOCITY_FACTOR_RADIANS_PER_SECOND_PER_RPM = (2 * Math.PI) / 60.0; // radians per second, per RPM
		
		// public static final double TURNING_P = .38;
    	// public static final double TURNING_I = 0.001;
    	// public static final double TURNING_D = 1;
		public static final double TURNING_P = .38;
    	public static final double TURNING_I = 0;
    	public static final double TURNING_D = 0;
    	public static final double TURNING_FF = 0;

		public static final double TURNING_MIN_OUTPUT_NORMALIZED = -1;
		public static final double TURNING_MAX_OUTPUT_NORMALIZED = 1;

		public static final double MAX_VELOCITY = 2 * 60; //radians per second (conversion factor is in radians and velocity is typically measured in rotations per minute so we multiply by 60)
		public static final double MAX_ACCELERATION_FACTOR = 2; // This will get multiplied by the MAX_VELOCITY and will in up being in radians/sec/sec Good rule of thumb is to set max accel to twice the max velocity.
		public static final double ALLOWED_ERROR = Rotation2d.fromDegrees(0.5).getRadians();

		public static final double SOFT_LIMIT_FORWARD = Rotation2d.fromRadians(2.1).getRadians();
		public static final double SOFT_LIMIT_REVERSE = 0.028;
	}

	public static final class TelescopingArmConstants {
		public static final IdleMode IDLE_MODE = IdleMode.kBrake;
		public static final int CURRENT_LIMIT_AMPS = 20; // amps

		public static final double TURNING_P = 1.0; // 1.0 might be a bit too much - reduce a bit if needed
		public static final double TURNING_I = 0;
		public static final double TURNING_D = 0;
		public static final double TURNING_FF = 0;
		public static final double TURNING_MIN_OUTPUT_NORMALIZED = -1;
		public static final double TURNING_MAX_OUTPUT_NORMALIZED = 1;

		public static final double MAX_VELOCITY = 2;
		public static final double MAX_ACCELERATION_FACTOR = 2; 
		public static final double ALLOWED_ERROR = .1;

		public static final double KICK_IT_STATE = 10; //Rotations required to release the tele arm enough to kick the ball into the spinning wheels

		// Soft limits for telescoping arm position (in rotations)
		public static final double SOFT_LIMIT_FORWARD = 5.0;  // Maximum extended position
		public static final double SOFT_LIMIT_REVERSE = 0.0;  // Minimum retracted position
	}

	public static final class CoralatorConstants {
		public static final double INTAKE_SPEED = -0.05;
		public static final double EJECT_SPEED = 0.05;
		public static final int CURRENT_LIMIT_AMPS = 60;
		public static final IdleMode MOTOR_IDLE_MODE = IdleMode.kBrake;
	}

	public static final class AlgaeBlasterConstants {
		public static final double INTAKE_SPEED = -1.0;
		public static final double EJECT_SPEED = 0.55;
		public static final int CURRENT_LIMIT_AMPS = 60;
		public static final IdleMode MOTOR_IDLE_MODE = IdleMode.kBrake;
	}
}