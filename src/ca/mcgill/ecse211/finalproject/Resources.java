package ca.mcgill.ecse211.finalproject;

import java.math.BigDecimal;
import java.util.Map;

import ca.mcgill.ecse211.finalproject.phase1.UltrasonicLocalizer;
import ca.mcgill.ecse211.finalproject.phase2.ColorPoller;
import ca.mcgill.ecse211.finalproject.phase2.Odometer;
import ca.mcgill.ecse211.finalproject.phase2.PathFinder;
//import ca.mcgill.ecse211.finalproject.phase2.PathFinder;
//import ca.mcgill.ecse211.finalproject.UltrasonicPoller;
//import ca.mcgill.ecse211.finalproject.phase1.UltrasonicLocalizerDisplay;
/** WIFI package stuff */
import ca.mcgill.ecse211.wificlient.WifiConnection;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

/**
 * This class is used to define static resources in one place for easy access
 * and to avoid cluttering the rest of the code base. All resources can be
 * imported at once like this:
 * 
 * {@code import static ca.mcgill.ecse211.lab3.Resources.*;}
 */
public class Resources {
	// ------------------CONSTANTS--------------------
	/**
	 * The dimensions sizes in squares
	 */
	public static final int ARENA_X = 8;
	public static final int ARENA_Y = 8;

	/**
	 * The threshold in intensity delta for determining when a line has been
	 * detected
	 */
	public static final double INTENSITY_THRESHOLD = 0.07f;

	/**
	 * Distance to light sensor from center of rotation in centimeters
	 */
	public static final double SENSOR_RADIUS = 5d;

	/**
	 * The wheel radius in centimeters.
	 */
	public static final double WHEEL_RAD = 2.13;

	/**
	 * The robot width in centimeters.
	 */
	public static final double TRACK = 11.90; // plis dont touch

	/**
	 * The lower speed at which the robot moves towards a black line
	 */
	public static final int LOW_FORWARD_SPEED = 110; // TODO: optimize this
	
	/**
	 * the higher speed at which the robot moves away from a black line
	 */
	public static final int HIGH_FORWARD_SPEED = 220;

	/**
	 * The speed at which the robot rotates in degrees per second.
	 */
	public static final int ROTATE_SPEED = 140;

	/**
	 * Low speed used for more accurate sensor radians
	 */
	public static final int CORRECTION_SPPED = 45;

	/**
	 * The motor acceleration in degrees per second squared.
	 */
	public static final int ACCELERATION = 1000;

	/**
	 * The tile size in centimeters.
	 */
	public static final double TILE_SIZE = 30.48;

	/**
	 * speed for the launcher motor
	 */
	public static final int SHOOTER_MOTOR_SPEED = 300;

	/**
	 * obstacle detection threshold
	 */
	public static final double ObstacleDetectionThreashold = 12.0;

	// ------------------EV3 Components--------------------
	/**
	 * The left motor.
	 */
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));

	/**
	 * The right motor.
	 */
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

	/**
	 * motor for the ball launcher
	 */
	public static final EV3LargeRegulatedMotor shooterMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

	/**
	 * The LCD.
	 */
	public static final TextLCD LCD = LocalEV3.get().getTextLCD();

	/**
	 * display
	 */
	public static final Display display = new Display();

	/**
	 * us display
	 */
	// public static final UltrasonicLocalizerDisplay USdisplay = new
	// UltrasonicLocalizerDisplay();

	/**
	 * The ultrasonic sensor.
	 */
	public static final EV3UltrasonicSensor US_SENSOR = new EV3UltrasonicSensor(SensorPort.S3);

	/**
	 * left color sensor
	 */
	public static final EV3ColorSensor leftColorSensor = new EV3ColorSensor(SensorPort.S1);

	/**
	 * right color sensor
	 */
	public static final EV3ColorSensor rightColorSensor = new EV3ColorSensor(SensorPort.S2);

	// ------------------Software Components--------------------

	/**
	 * The ultrasonic poller.
	 */
	public static final UltrasonicPoller usPoller = new UltrasonicPoller();

	/**
	 * US localizer
	 */
	public static final UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer();

	/**
	 * colorPoller
	 */
	public static final ColorPoller colorPoller = new ColorPoller();

	/**
	 * Navigation
	 */
	public static final Navigation navigation = new Navigation();

	/**
	 * The odometer.
	 */
	public static Odometer odometer = Odometer.getOdometer();

	/** ---------------------------- WIFI PACKAGE ---------------------------- */
	/**
	 * Integrate this carefully with your existing Resources class (See below for
	 * where to add your code from your current Resources file). The order in which
	 * things are declared matters!
	 * 
	 * When you're done, replace this javadoc comment with the one you have from
	 * your Resources class.
	 * 
	 * @author Younes Boubekeur
	 */

	// Set these as appropriate for your team and current situation
	/**
	 * The default server IP used by the profs and TA's.
	 */
	public static final String DEFAULT_SERVER_IP = "192.168.2.3";

	/**
	 * The IP address of the server that transmits data to the robot. Set this to
	 * the default for the beta demo and competition.
	 */
	public static final String SERVER_IP = "192.168.2.32";

	/**
	 * Your team number.
	 */
	public static final int TEAM_NUMBER = 9;

	/**
	 * Enables printing of debug info from the WiFi class.
	 */
	public static final boolean ENABLE_DEBUG_WIFI_PRINT = true;

	/**
	 * Enable this to attempt to receive Wi-Fi parameters at the start of the
	 * program.
	 */
	public static final boolean RECEIVE_WIFI_PARAMS = true;

	// DECLARE YOUR CURRENT RESOURCES HERE
	// eg, motors, sensors, constants, etc
	//////////////////////////////////////

	/**
	 * Container for the Wi-Fi parameters.
	 */
	public static Map<String, Object> wifiParameters;

	// Used to receive the parameters from the server.
	static {
		if (RECEIVE_WIFI_PARAMS) {
			receiveWifiParameters();
		}
	}
	/**
	 * Red team number.
	 */
	public static int redTeam = get("RedTeam");

	/**
	 * Red team's starting corner.
	 */
	public static int redCorner = get("RedCorner");

	/**
	 * Green team number.
	 */
	public static int greenTeam = get("GreenTeam");

	/**
	 * Green team's starting corner.
	 */
	public static int greenCorner = get("GreenCorner");

	/**
	 * The Red Zone.
	 */
	public static Region red = new Region("Red_LL_x", "Red_LL_y", "Red_UR_x", "Red_UR_y");

	/**
	 * The Green Zone.
	 */
	public static Region green = new Region("Green_LL_x", "Green_LL_y", "Green_UR_x", "Green_UR_y");

	/**
	 * The Island.
	 */
	public static Region island = new Region("Island_LL_x", "Island_LL_y", "Island_UR_x", "Island_UR_y");

	/**
	 * The red tunnel footprint.
	 */
	// public static Region tnr = new Region("TNR_LL_x", "TNR_LL_y", "TNR_UR_x",
	// "TNR_UR_y");
	public static double targetAngle = Math.max(get("TNR_LL_x"), get("TNR_UR_x"));

	/**
	 * The green tunnel footprint.
	 */
	public static Region tng = new Region("TNG_LL_x", "TNG_LL_y", "TNG_UR_x", "TNG_UR_y");

	/**
	 * The location of the target bin.
	 */
	public static Point bin = new Point(get("BIN_x"), get("BIN_y"));

	/**
	 * Receives Wi-Fi parameters from the server program.
	 */
	public static void receiveWifiParameters() {
		// Only initialize the parameters if needed
		if (!RECEIVE_WIFI_PARAMS || wifiParameters != null) {
			return;
		}
		// System.out.println("Waiting to receive Wi-Fi parameters.");

		// Connect to server and get the data, catching any errors that might occur
		try (WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT)) {
			/*
			 * getData() will connect to the server and wait until the user/TA presses the
			 * "Start" button in the GUI on their laptop with the data filled in. Once it's
			 * waiting, you can kill it by pressing the upper left hand corner button
			 * (back/escape) on the EV3. getData() will throw exceptions if it can't connect
			 * to the server (e.g. wrong IP address, server not running on laptop, not
			 * connected to WiFi router, etc.). It will also throw an exception if it
			 * connects but receives corrupted data or a message from the server saying
			 * something went wrong. For example, if TEAM_NUMBER is set to 1 above but the
			 * server expects teams 17 and 5, this robot will receive a message saying an
			 * invalid team number was specified and getData() will throw an exception
			 * letting you know.
			 */
			wifiParameters = conn.getData();
		} catch (Exception e) {
			System.err.println("Error: " + e.getMessage());
		}
	}

	/**
	 * Test path finder TODO: the way the path finder is initialized is subject to
	 * change
	 */
	public static PathFinder pathFinder; //YP: not final because I want to construct this after localization

	/**
	 * Returns the Wi-Fi parameter int value associated with the given key.
	 * 
	 * @param key the Wi-Fi parameter key
	 * @return the Wi-Fi parameter int value associated with the given key
	 */
	public static int get(String key) {
		if (wifiParameters != null) {
			return ((BigDecimal) wifiParameters.get(key)).intValue();
		} else {
			return 0;
		}
	}

	/**
	 * Represents a region on the competition map grid, delimited by its lower-left
	 * and upper-right corners (inclusive).
	 * 
	 * @author Younes Boubekeur
	 */
	public static class Region {
		/** The lower left corner of the region. */
		public Point ll;

		/** The upper right corner of the region. */
		public Point ur;

		/**
		 * Constructs a Region.
		 * 
		 * @param lowerLeft  the lower left corner of the region
		 * @param upperRight the upper right corner of the region
		 */
		public Region(Point lowerLeft, Point upperRight) {
			validateCoordinates(lowerLeft, upperRight);
			ll = lowerLeft;
			ur = upperRight;
		}

		/**
		 * Helper constructor to make a Region directly from parameter names.
		 * 
		 * @param llX the Wi-Fi parameter key representing the lower left corner of the
		 *            region x coordinate
		 * @param llY the Wi-Fi parameter key representing the lower left corner of the
		 *            region y coordinate
		 * @param urX the Wi-Fi parameter key representing the upper right corner of the
		 *            region x coordinate
		 * @param urY the Wi-Fi parameter key representing the upper right corner of the
		 *            region y coordinate
		 */
		public Region(String llX, String llY, String urX, String urY) {
			this(new Point(get(llX), get(llY)), new Point(get(urX), get(urY)));
		}

		/**
		 * Validates coordinates.
		 * 
		 * @param lowerLeft  the lower left corner of the region
		 * @param upperRight the upper right corner of the region
		 */
		private void validateCoordinates(Point lowerLeft, Point upperRight) {
			if (lowerLeft.x > upperRight.x || lowerLeft.y > upperRight.y) {
				throw new IllegalArgumentException("Upper right cannot be below or to the left of lower left!");
			}
		}

		public String toString() {
			return "[" + ll + ", " + ur + "]";
		}
	}

	/**
	 * Represents a coordinate point on the competition map grid.
	 * 
	 * @author Younes Boubekeur
	 */
	public static class Point {
		/** The x coordinate. */
		public double x;

		/** The y coordinate. */
		public double y;

		/**
		 * Constructs a Point.
		 * 
		 * @param x the x coordinate
		 * @param y the y coordinate
		 */
		public Point(double x, double y) {
			this.x = x;
			this.y = y;
		}

		public String toString() {
			return "(" + x + ", " + y + ")";
		}

	}
}
