package frc.robot;

/**
 * Contains the definitions of all the ports
 */
public class Ports {

		// IP (v4) addresses
		// The purpose of this section is to serve as a reminder of what static IP (v4) addresses are used so they are consistent
		// between the competition and practice robots.
		//
		// The radio is automatically set to 10.12.25.1
		// The Rio is set to static 10.12.25.2, mask 255.255.255.0
		// The Limelight is set to 10.12.25.11, mask 255.255.255.0, gateway 10.12.25.1
		// but note that pressing the reset button will revert to DHCP.
		// The Raspberry Pi running FRCVision is set to static 10.12.25.12, mask 255.255.255.0, gateway 10.12.25.1, DNS blank
		//
		// If a device cannot be accessed (e.g. because its address was somehow obtained via DHCP and mDNS is not working),
		// use Angry IP Scanner to find it!


		/**
		 * Digital ports
		 */
		public static class Digital {
		}
		
		/**
		 * Analog ports
		 */
		public static class Analog {
			public static final int FRONT_RIGHT_TURNING_ABSOLUTE_ENCODER = 0;
			public static final int REAR_RIGHT_TURNING_ABSOLUTE_ENCODER = 1;
			public static final int REAR_LEFT_TURNING_ABSOLUTE_ENCODER = 2;
			public static final int FRONT_LEFT_TURNING_ABSOLUTE_ENCODER = 3;			
		}
		
		/**
		 * Relays
		 */
		public static class Relay {
			public static final int COMPRESSOR_RELAY = 0;
		}
		
		/**
		 * CAN Ids
		 */
		public static class CAN {
			// Pneumatic Control Module
			public static final int PCM = 12;
			//2025 Robot
			public static final int RIO = 0;
			public static final int PDP = 1;	

			public static final int pigeon2 = 2;

			// SPARK MAX CAN IDs
			public static final int FRONT_LEFT_DRIVING = 10;
			public static final int REAR_LEFT_DRIVING = 8;
			public static final int FRONT_RIGHT_DRIVING = 4;
			public static final int REAR_RIGHT_DRIVING = 6;

			public static final int FRONT_LEFT_TURNING = 9;
			public static final int REAR_LEFT_TURNING = 7;
			public static final int FRONT_RIGHT_TURNING = 3;
			public static final int REAR_RIGHT_TURNING = 5;

		}
		
		/**
		 * USB ports
		 */
		public static class USB {
			public static final int RIGHT_JOYSTICK = 4;
			public static final int LEFT_JOYSTICK = 2;
			public static final int COPILOT_GAMEPAD = 1;
			public static final int MAIN_JOYSTICK = 0;
		}
		
		/**
		 * PCM ports
		 */
		public static class PCM {
			public static final int ALGAE_BLASTER_BLAST = 0;
			public static final int ALGAE_BLASTER_RESET = 1;
		}

		/**
		 * PWM ports
		 */
		public static class PWM {
			public static final int LED_STRIP = 9;
		}

		/**
		 * USB cameras
		 */
		public static class UsbCamera {
			public static final int FLOOR_CAMERA = 0;
			public static final int SHOOTER_CAMERA = 1;
			public static final int TOP_CAMERA = 2;
		}
}
