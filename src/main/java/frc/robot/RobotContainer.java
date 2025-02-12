// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;

import frc.robot.sensors.*;

import frc.robot.subsystems.SwerveDrivetrain;

import frc.robot.subsystems.Indicator;

import frc.robot.commands.drivetrain.*;
import frc.robot.interfaces.ICamera;
import frc.robot.commands.indicator.*;
import frc.robot.commands.groups.*;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

	public static final double GAMEPAD_AXIS_THRESHOLD = 0.15;
	public static final double JOYSTICK_AXIS_THRESHOLD = 0.15;

	public static final int LX = 0;
	public static final int LY = 1;
	public static final int LT = 2;
	public static final int RT = 3;
	public static final int RX = 4;
	public static final int RY = 5;

	Command indicatorTimedScrollRainbow; // command to run while stating up and when disabled

	// choosers (for auton)
	
	public static final String AUTON_DO_NOTHING = "Do Nothing";
	public static final String AUTON_CUSTOM = "My Auto";
	public static final String AUTON_SAMPLE_SWERVE = "Sample Swerve";
	public static final String AUTON_SAMPLE_MOVE_FORWARD = "Sample Move Forward";
	public static final String AUTON_SAMPLE_MOVE_IN_REVERSE = "Sample Move In Reverse";
	public static final String AUTON_SAMPLE_MOVE_IN_GAMMA_SHAPE = "Sample Move In Gamma Shape";
	public static final String AUTON_SAMPLE_MOVE_IN_L_SHAPE_IN_REVERSE = "Sample Move In L Shape In Reverse";
	public static final String AUTON_TEST_HARDCODED_MOVE_1 = "Test Hardcoded Move 1";
	public static final String AUTON_TEST_HARDCODED_MOVE_2 = "Test Hardcoded Move 2";
	public static final String AUTON_TEST_TRAJECTORY_GENERATION = "Test Trajectory Generation";
	private String autonSelected;
	private SendableChooser<String> autonChooser = new SendableChooser<>();

	public static final String GAME_PIECE_NONE = "None";
	public static final String GAME_PIECE_1_NOTE = "1 Note";
	public static final String GAME_PIECE_2_NOTES = "2 Notes";
	public static final String GAME_PIECE_3_NOTES = "3 Notes";
	private String gamePieceSelected;
	private SendableChooser<String> gamePieceChooser = new SendableChooser<>();
	
	public static final String START_POSITION_1 = "Starting Position 1";
	public static final String START_POSITION_2 = "Starting Position 2";
	public static final String START_POSITION_3 = "Starting Position 3";
	public static final String START_POSITION_4 = "Starting Position 4";
	public static final String START_POSITION_5 = "Starting Position 5";
	public static final String START_POSITION_6 = "Starting Position 6";
	private String startPosition;
	private SendableChooser<String> startPositionChooser = new SendableChooser<>();

	public static final String MAIN_TARGET_SPEAKER = "Speaker";
	public static final String MAIN_TARGET_NOWHERE = "Nowhere";
	private String mainTarget;
	private SendableChooser<String> mainTargetChooser = new SendableChooser<>();
	
	public static final String CAMERA_OPTION_USE_ALWAYS = "Always";
	public static final String CAMERA_OPTION_USE_OPEN_LOOP_ONLY = "Open Loop Only";
	public static final String CAMERA_OPTION_USE_CLOSED_LOOP_ONLY = "Closed Loop Only";
	public static final String CAMERA_OPTION_USE_NEVER = "Never";
	private String cameraOption;
	private SendableChooser<String> cameraOptionChooser = new SendableChooser<>();
	
	public static final String SONAR_OPTION_USE_ALWAYS = "Always";
	public static final String SONAR_OPTION_USE_RELEASE_ONLY = "Release Only";
	public static final String SONAR_OPTION_USE_GRASP_ONLY = "Grasp Only";
	public static final String SONAR_OPTION_USE_NEVER = "Never";
	private String sonarOption;
	private SendableChooser<String> sonarOptionChooser = new SendableChooser<>();
	
	public static final String CLAW_OPTION_RELEASE = "Release";
	public static final String CLAW_OPTION_DONT_RELEASE = "Don't Release"; 
	private String releaseSelected;
	private SendableChooser<String> releaseChooser = new SendableChooser<>();

	public static final String AUTON_OPTION_JUST_SHOOT_NOTE = "Just Shoot Note";
	public static final String AUTON_OPTION_LEAVE_COMMUNITY = "Leave Community";
	public static final String AUTON_OPTION_PICKUP_NOTE_AT_MIDLINE = "Pickup Note at Midline";
	public static final String AUTON_OPTION_PICKUP_NOTE_AT_WING = "Pickup Note at Wing";
	public static final String AUTON_OPTION_FEED_NOTE = "Feed Note";
	private String autonOption;
	private SendableChooser<String> autonOptionChooser = new SendableChooser<>();

	// sensors

	private final HMAccelerometer accelerometer = new HMAccelerometer();

	private final ICamera object_detection_camera = new ObjectDetectionCamera();

	private final ICamera apriltag_camera = new AprilTagCamera();

	// motorized devices

	private final SwerveDrivetrain drivetrain = new SwerveDrivetrain();

	/*private final WPI_TalonSRX drawer_master = new WPI_TalonSRX(Ports.CAN.DRAWER);

	private final Drawer drawer = new Drawer(drawer_master);*/

	// private final WPI_TalonFX elevator_master = new WPI_TalonFX(Ports.CAN.ELEVATOR_MASTER);
	// private final WPI_TalonFX elevator_follower = new WPI_TalonFX(Ports.CAN.ELEVATOR_FOLLOWER);

	// private final /*I*/Elevator elevator = new Elevator(elevator_master, elevator_follower);

	// private final WPI_TalonFX neck_master = new WPI_TalonFX(Ports.CAN.NECK_MASTER);
	// private final WPI_TalonFX neck_follower = new WPI_TalonFX(Ports.CAN.NECK_FOLLOWER);
	
	// private final /*I*/Neck neck = new Neck(neck_master, neck_follower);

	//private final CANSparkMax roller_master = new CANSparkMax(Ports.CAN.ROLLER, MotorType.kBrushless);
	// private final WPI_TalonSRX roller_master = new WPI_TalonSRX(Ports.CAN.ROLLER_MASTER);
	// private final WPI_TalonSRX roller_follower = new WPI_TalonSRX(Ports.CAN.ROLLER_FOLLOWER);

	// private final /*I*/Roller roller = new Roller(roller_master, roller_follower);

	//private final CANSparkMax shooter_master = new CANSparkMax(Ports.CAN.SHOOTER_MASTER, MotorType.kBrushless);
	//private final CANSparkMax shooter_follower = new CANSparkMax(Ports.CAN.SHOOTER_FOLLOWER, MotorType.kBrushless);

	//private final /*I*/SimpleShooter shooter = new SimpleShooter(shooter_master, shooter_follower);

	// private final WPI_TalonFX shooter_master = new WPI_TalonFX(Ports.CAN.SHOOTER_MASTER);
	
	// private final /*I*/Shooter shooter = new Shooter(shooter_master);

	// pneumatic devices

	//private final Compressor compressor = new Compressor();

	//private final Mouth mouth = new Mouth();

	// misc

	private final Field2d field = new Field2d(); //  a representation of the field

	private final Indicator indicator = new Indicator(apriltag_camera, object_detection_camera);

	// The driver's and copilot's joystick(s) and controller(s)

	/*CommandJoystick joyLeft = new CommandJoystick(Ports.USB.LEFT_JOYSTICK);
	CommandJoystick joyRight = new CommandJoystick(Ports.USB.RIGHT_JOYSTICK);*/
	CommandJoystick joyMain = new CommandJoystick(Ports.USB.MAIN_JOYSTICK);
	//CommandXboxController driverGamepad = new CommandXboxController(Ports.USB.DRIVER_GAMEPAD);
	CommandXboxController copilotGamepad = new CommandXboxController(Ports.USB.COPILOT_GAMEPAD);
	

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		// choosers (for auton)
		
		autonChooser.setDefaultOption("Do Nothing", AUTON_DO_NOTHING);
		autonChooser.addOption("My Auto", AUTON_CUSTOM);
		autonChooser.addOption("Sample Swerve", AUTON_SAMPLE_SWERVE);
		autonChooser.addOption("Sample Move Forward", AUTON_SAMPLE_MOVE_FORWARD);
		autonChooser.addOption("Sample Move In Reverse", AUTON_SAMPLE_MOVE_IN_REVERSE);
		autonChooser.addOption("Sample Move In Gamma Shape", AUTON_SAMPLE_MOVE_IN_GAMMA_SHAPE);
		autonChooser.addOption("Sample Move In L Shape In Reverse", AUTON_SAMPLE_MOVE_IN_L_SHAPE_IN_REVERSE);
		autonChooser.addOption("Test Hardcoded Move 1", AUTON_TEST_HARDCODED_MOVE_1);
		autonChooser.addOption("Test Hardcoded Move 2", AUTON_TEST_HARDCODED_MOVE_2);
		autonChooser.addOption("Test Trajectory Generation", AUTON_TEST_TRAJECTORY_GENERATION);
		SmartDashboard.putData("Auto choices", autonChooser);

		gamePieceChooser.setDefaultOption("None", GAME_PIECE_NONE);
		gamePieceChooser.addOption("1 Note", GAME_PIECE_1_NOTE);
		gamePieceChooser.addOption("2 Notes", GAME_PIECE_2_NOTES);
		gamePieceChooser.addOption("3 Notes", GAME_PIECE_3_NOTES);
		SmartDashboard.putData("Game piece choices", gamePieceChooser);

		startPositionChooser.setDefaultOption("Starting Position 1", START_POSITION_1);
		startPositionChooser.addOption("Starting Position 2", START_POSITION_2);
		startPositionChooser.addOption("Starting Position 3", START_POSITION_3);
		startPositionChooser.addOption("Starting Position 4", START_POSITION_4);
		startPositionChooser.addOption("Starting Position 5", START_POSITION_5);
		startPositionChooser.addOption("Starting Position 6", START_POSITION_6);
		SmartDashboard.putData("Start positions", startPositionChooser);

		mainTargetChooser.setDefaultOption("To Nowhere", MAIN_TARGET_NOWHERE);
		mainTargetChooser.addOption("Speaker", MAIN_TARGET_SPEAKER);
		SmartDashboard.putData("Main targets", mainTargetChooser);
		
		cameraOptionChooser.setDefaultOption("Always", CAMERA_OPTION_USE_ALWAYS);
		cameraOptionChooser.addOption("Open Loop Only", CAMERA_OPTION_USE_OPEN_LOOP_ONLY);
		cameraOptionChooser.addOption("Closed Loop Only", CAMERA_OPTION_USE_CLOSED_LOOP_ONLY);
		cameraOptionChooser.addOption("Never", CAMERA_OPTION_USE_NEVER);		
		SmartDashboard.putData("Camera options", cameraOptionChooser);
		
		sonarOptionChooser.setDefaultOption("Always", SONAR_OPTION_USE_ALWAYS);
		sonarOptionChooser.addOption("Release Only", SONAR_OPTION_USE_RELEASE_ONLY);
		sonarOptionChooser.addOption("Grasp Only", SONAR_OPTION_USE_GRASP_ONLY);		
		sonarOptionChooser.addOption("Never", SONAR_OPTION_USE_NEVER);
		SmartDashboard.putData("Sonar options", sonarOptionChooser);
		
		releaseChooser.setDefaultOption("Release", CLAW_OPTION_RELEASE);
		releaseChooser.addOption("Don't release", CLAW_OPTION_DONT_RELEASE);
		SmartDashboard.putData("Release options", releaseChooser);

		autonOptionChooser.setDefaultOption("Just Shoot Note", AUTON_OPTION_JUST_SHOOT_NOTE);
		autonOptionChooser.addOption("Leave Community", AUTON_OPTION_LEAVE_COMMUNITY);
		autonOptionChooser.addOption("Pickup Note At Midline", AUTON_OPTION_PICKUP_NOTE_AT_MIDLINE);
		autonOptionChooser.addOption("Pickup Note At Wing", AUTON_OPTION_PICKUP_NOTE_AT_WING);
		autonOptionChooser.addOption("Feed Note", AUTON_OPTION_FEED_NOTE);
	
		SmartDashboard.putData("Auton options", autonOptionChooser);
		

		// Configure the button bindings

		configureButtonBindings();


		// Configure default commands

		drivetrain.setDefaultCommand(
			// The left stick controls translation of the robot.
			// Turning is controlled by the X axis of the right stick.
			// We are inverting LeftY because Xbox controllers return negative values when we push forward.
			// We are inverting LeftX because we want a positive value when we pull to the left. Xbox controllers return positive values when you pull to the right by default.
			// We are also inverting RightX because we want a positive value when we pull to the left (CCW is positive in mathematics).
			new RunCommand(
				() -> drivetrain.drive(
					-MathUtil.applyDeadband(joyMain.getY(), JOYSTICK_AXIS_THRESHOLD),
					-MathUtil.applyDeadband(joyMain.getX(), JOYSTICK_AXIS_THRESHOLD),
					-MathUtil.applyDeadband(joyMain.getRawAxis(4), JOYSTICK_AXIS_THRESHOLD),
					true, true),
				drivetrain));

		indicator.setDefaultCommand(new IndicatorIndicateUsingCamera(indicator)); // default command, only runs when robot is enabled

		indicatorTimedScrollRainbow = new IndicatorTimedScrollRainbow(indicator,1);
		indicatorTimedScrollRainbow.schedule(); // we schedule the command as we are starting up
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
	 * subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
	 * passing it to a
	 * {@link JoystickButton}.
	 */
	private void configureButtonBindings() {

		// driver (joystick)

		joyMain.povUp()
			.onTrue(new DrivetrainZeroHeading(drivetrain));	

		joyMain.povDown()
			.onTrue(new DrivetrainOppositeHeading(drivetrain));	

		joyMain.povLeft()
			.onTrue(new DrivetrainLeftSubHeading(drivetrain));	

		joyMain.povRight()
			.onTrue(new DrivetrainRightSubHeading(drivetrain));

		joyMain.button(1)
			.whileTrue(new DrivetrainDriveUsingAprilTagCamera(drivetrain, apriltag_camera, getMainJoystick()));

		joyMain.button(2)
			//.whileTrue(new DrivetrainSetXFormation(drivetrain));	
			.whileTrue(new DrivetrainDriveUsingObjectDetectionCamera(drivetrain, object_detection_camera, getMainJoystick()));
			
		// joyMain.button(3)
		// 	.onTrue(new MoveInLShapeInReverse(drivetrain, this, 3));
			
		// joyMain.button(4)
		// 	.onTrue(new MoveInGammaShape(drivetrain, this, 3));

		// joyMain.button(5)
		// 	.onTrue(new MoveForward(drivetrain, this, 3));
			//.onTrue(new DrivetrainTurnAngleUsingPidController(drivetrain, -90));
			//.onTrue(new MoveInUShapeInReverse(drivetrain, this, 1));

		joyMain.button(6)
			//.onTrue(new MoveInReverse(drivetrain, this, 3));
			//.onTrue(new DrivetrainTurnAngleUsingPidController(drivetrain, 90));
			//.onTrue(new DrivetrainTurnUsingCamera(drivetrain, object_detection_camera));
			.whileTrue(new DrivetrainSetXFormation(drivetrain));

		// joyMain.button(7)
		// 	.whileTrue(new RollerJoystickControl(roller, drivetrain, getMainJoystick()));
		
		// joyMain.button(8)
		// 	.whileTrue(new NeckJoystickControl(neck, drivetrain, getMainJoystick()));
		
		// joyMain.button(9)

		// 	.whileTrue(new ShooterJoystickControl(shooter, drivetrain, getMainJoystick()));
		
		// joyMain.button(10)
		// 	.whileTrue(new ElevatorJoystickControl(elevator, drivetrain, getMainJoystick()));

		joyMain.button(11)
			//.onTrue(new DrivetrainZeroHeading(drivetrain));
			.onTrue(new DrivetrainTurnUsingCamera(drivetrain, apriltag_camera));
		
		joyMain.button(12)
			//.whileTrue(new DrivetrainSetXFormation(drivetrain));
			.onTrue(new DrivetrainTurnUsingCamera(drivetrain, object_detection_camera));
			
				
		// copilot (gamepad)
		
		// copilotGamepad.a()
		// 	.whileTrue(new RollerRelease(roller));
		
		// copilotGamepad.b()
		// 	.whileTrue(new RollerRoll(roller));

		// copilotGamepad.x()
		// 	//.onTrue(new MouthSafeClose(mouth, neck, getCopilotGamepad()));
		// 	//.onTrue(new RollerRollUntilNoteSensed(roller, getNoteSensor()));
		// 	//.onTrue(new RollerSmartRoll(roller, noteSensor));
		// 	//.onTrue(new RollerRollLowRpmUntilNoteSensed(roller, getNoteSensor()));
		// 	//.onTrue(new RollerSuperSmartRoll(roller, noteSensor));
		// 	.onTrue(new RollerReleaseShortDistance(roller));

		// copilotGamepad.y()
		// 	//.whileTrue(new RollerRollLowRpm(roller));
		// 	//.onTrue(new RollerRollLowRpmUntilNoteSensed(roller, getNoteSensor()));
		// 	//.onTrue(new RollerReleaseShortDistance(roller));
		// 	//.onTrue(new RollerSuperSmartRoll(roller, noteSensor, noteSensorTwo));
		// 	.onTrue(new RollerRollLowRpmUntilNoteSensed(roller, noteSensor, noteSensorTwo));
			
		// copilotGamepad.back()
		// 	//.onTrue(new DrivetrainAndGyroReset(drivetrain));
		// 	.onTrue(new AlmostEverythingStop(elevator, neck, roller, shooter));

		// copilotGamepad.start()
		// 	//.onTrue(new AlmostEverythingStop(elevator, neck, roller));
		// 	.onTrue(new NeckHome(neck));


		// copilotGamepad.leftTrigger()
		// 	//.onTrue(new DrawerRetractWithStallDetection(drawer));
		// 	//.whileTrue(new ShooterTake(shooter));
		// 	.whileTrue(new ShooterShootHigh(shooter));

		// copilotGamepad.rightTrigger()
		// 	//.onTrue(new DrawerExtendWithStallDetection(drawer));
		// 	.whileTrue(new RollerRoll(roller));


		// copilotGamepad.povDown()
		// 	//.onTrue(new ElevatorMoveDownWithStallDetection(elevator));
		// 	.onTrue(new NeckMoveDownWithStallDetection(neck));

		// copilotGamepad.povLeft()
		// 	//.onTrue(new ElevatorMoveMidwayWithStallDetection(elevator));
		// 	.onTrue(new NeckMoveSubWithStallDetection(neck));

		// copilotGamepad.povRight()
		// 	//.onTrue(new ElevatorMoveMidwayWithStallDetection(elevator));
		// 	//.onTrue(new NeckMovePodiumWithStallDetection(neck));
		// 	.onTrue(new NeckMoveFeedNoteWithStallDetection(neck));

		// copilotGamepad.povUp()
		// 	//.onTrue(new ElevatorMoveUpWithStallDetection(elevator));
		// 	.onTrue(new NeckMoveUpWithStallDetection(neck));


		// copilotGamepad.leftBumper()
		// 	//.onTrue(new NeckMoveUpWithStallDetection(neck));
		// 	//.onTrue(new NeckMoveUpWithStallDetection(neck));
		// 	.whileTrue(new NeckMoveUsingCamera(neck, apriltag_camera));

		// copilotGamepad.rightBumper()
		// 	//.onTrue(new NeckMoveDownWithStallDetection(neck));
		// 	.onTrue(new NeckMoveAcrossFieldWithStallDetection(neck));


		// copilotGamepad.leftStick()
		// 	.onTrue(new RollerTimedRoll(roller, 3));
		// 	//.onTrue(new GamepadRumble(getCopilotGamepad(),false));			

		// copilotGamepad.rightStick()
		// 	.onTrue(new RollerTimedRelease(roller, 3));
		// 	//.onTrue(new GamepadRumble(getCopilotGamepad(),false));


		// copilotGamepad.axisGreaterThan(LY,GAMEPAD_AXIS_THRESHOLD)
		// 	.whileTrue(new ElevatorGamepadControl(elevator, getCopilotGamepad()));

		// copilotGamepad.axisLessThan(LY,-GAMEPAD_AXIS_THRESHOLD)
		// 	.whileTrue(new ElevatorGamepadControl(elevator, getCopilotGamepad()));

		/*copilotGamepad.axisGreaterThan(LX,GAMEPAD_AXIS_THRESHOLD)
			.whileTrue();

		copilotGamepad.axisLessThan(LX,-GAMEPAD_AXIS_THRESHOLD)
			.whileTrue();*/

		// copilotGamepad.axisGreaterThan(RY,GAMEPAD_AXIS_THRESHOLD)
		// 	.whileTrue(new NeckGamepadControl(neck, getCopilotGamepad()));

		// copilotGamepad.axisLessThan(RY,-GAMEPAD_AXIS_THRESHOLD)
		// 	.whileTrue(new NeckGamepadControl(neck, getCopilotGamepad()));

		copilotGamepad.axisGreaterThan(RX,GAMEPAD_AXIS_THRESHOLD);
			//.whileTrue(new DrawerGamepadControl(drawer, getCopilotGamepad()));
			//.onTrue(new NeckMovePodiumWithStallDetection(neck));

		copilotGamepad.axisLessThan(RX,-GAMEPAD_AXIS_THRESHOLD);
			//.whileTrue(new DrawerGamepadControl(drawer, getCopilotGamepad()));
			//.onTrue(new NeckMoveSubWithStallDetection(neck));
			
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		autonSelected = autonChooser.getSelected();
		System.out.println("Auton selected: " + autonSelected);	

		gamePieceSelected = gamePieceChooser.getSelected();
		System.out.println("Game piece selected: " + gamePieceSelected);		

		startPosition = startPositionChooser.getSelected();
		System.out.println("Start position: " + startPosition);

		mainTarget = mainTargetChooser.getSelected();
		System.out.println("Main target: " + mainTarget);
		
		cameraOption = cameraOptionChooser.getSelected();
		System.out.println("Camera option: " + cameraOption);
		
		sonarOption = sonarOptionChooser.getSelected();
		System.out.println("Sonar option: " + sonarOption);
		
		releaseSelected = releaseChooser.getSelected();
		System.out.println("Release chosen: " + releaseSelected);

		autonOption = autonOptionChooser.getSelected();
		System.out.println("Auton option: " + autonOption);
		

		switch (autonSelected) {
			case AUTON_DO_NOTHING:
				return null;
				//break;
				
			default:
				// nothing
				return null;
				//break;
		} 
	}

	public TrajectoryConfig createFastTrajectoryConfig() {
		// Create config for trajectory
		TrajectoryConfig config = new TrajectoryConfig(
			AutoConstants.HIGH_SPEED_METERS_PER_SECOND,
			AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
			// Add kinematics to ensure max speed is actually obeyed
			.setKinematics(DrivetrainConstants.DRIVE_KINEMATICS);

		return config;
	}

	public TrajectoryConfig createTrajectoryConfig() {
		// Create config for trajectory
		TrajectoryConfig config = new TrajectoryConfig(
			AutoConstants.MAX_SPEED_METERS_PER_SECOND,
			AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
			// Add kinematics to ensure max speed is actually obeyed
			.setKinematics(DrivetrainConstants.DRIVE_KINEMATICS);

		return config;
	}

	public TrajectoryConfig createSlowTrajectoryConfig() {
		// Create config for trajectory
		TrajectoryConfig config = new TrajectoryConfig(
			AutoConstants.REDUCED_SPEED_METERS_PER_SECOND,
			AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
			// Add kinematics to ensure max speed is actually obeyed
			.setKinematics(DrivetrainConstants.DRIVE_KINEMATICS);

		return config;
	}


	public TrajectoryConfig createReverseTrajectoryConfig() {

		TrajectoryConfig config = createTrajectoryConfig();

		config.setReversed(true); // in reverse!

		return config;
	}

	public TrajectoryConfig createFastReverseTrajectoryConfig() {

		TrajectoryConfig config = createFastTrajectoryConfig();

		config.setReversed(true); // in reverse!

		return config;
	}

	public Field2d getField()
	{
		return field;
	}

	public HMAccelerometer getAccelerometer()
	{
		return accelerometer;
	}

	public ICamera getObjectDetectionCamera()
	{
		return object_detection_camera;
	}

	public ICamera getAprilTagCamera()
	{
		return apriltag_camera;
	}

	public SwerveDrivetrain getDrivetrain()
	{
		return drivetrain;
	}

	public Joystick getMainJoystick()
	{
		return joyMain.getHID();
	}

	public XboxController getCopilotGamepad()
	{
		return copilotGamepad.getHID();
	}

	public SendableChooser<String> getAutonChooser()
	{
		return autonChooser;
	}
	
	public SendableChooser<String> getGamePieceChooser()
	{
		return gamePieceChooser;
	}

	public SendableChooser<String> getStartPositionChooser()
	{
		return startPositionChooser;
	}

	public SendableChooser<String> getMainTargetChooser()
	{
		return mainTargetChooser;
	}

	public SendableChooser<String> getCameraOptionChooser()
	{
		return cameraOptionChooser;
	}

	public SendableChooser<String> getSonarOptionChooser()
	{
		return sonarOptionChooser;
	}

	public SendableChooser<String> getReleaseChooser()
	{
		return releaseChooser;
	}

	public SendableChooser<String> getAutonOptionChooser()
	{
		return autonOptionChooser;
	}
}
