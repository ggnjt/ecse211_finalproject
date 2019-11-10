package ca.mcgill.ecse211.finalproject;

import ca.mcgill.ecse211.finalproject.phase2.ColorPoller;
import ca.mcgill.ecse211.finalproject.phase2.Odometer;
import ca.mcgill.ecse211.finalproject.phase2.PathFinder;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;

/**
 * This class is used to define static resources in one place for easy access and to avoid cluttering the rest of the
 * code base. All resources can be imported at once like this:
 * 
 * {@code import static ca.mcgill.ecse211.lab3.Resources.*;}
 */
public class Resources {
  // ------------------CONSTANTS--------------------
  /**
   * The dimensions sizes in squares
   */
  public static final int ARENA_X = 14;
  public static final int ARENA_Y = 8;

  /**
   * The threshold for when the signal is considered to have triggered a rising or falling edge.
   */
  public final int SIGNAL_THRESHOLD = 50;

  /**
   * The threshold in intensity delta for determining when a line has been detected
   */
  public static final double INTENSITY_THRESHOLD = 0.05;

  /**
   * The threshold for the delta between the current and previous value of the ultrasonic sensor. Used to detect rising
   * and falling edges.
   */
  public static final int DELTA_THRESHOLD = 7;

  /**
   * The margin around NOISE_THRESHOLD to account for any noise in the signal.
   */
  public static final int NOISE_MARGIN = 3;

  /**
   * Distance to waypoint threshold in centimeters
   */
  public static final double WPOINT_RAD = 1;

  /**
   * Distance to light sensor from center of rotation in centimeters
   */
  public static final double SENSOR_RADIUS = 14.3d
      ;

  /**
   * The wheel radius in centimeters.
   */
  public static final double WHEEL_RAD = 2.13;

  /**
   * The robot width in centimeters.
   */
  public static final double TRACK = 12.3;

  /**
   * The speed at which the robot moves forward in degrees per second.
   */
  public static final int FORWARD_SPEED = 120;

  /**
   * The speed at which the robot rotates in degrees per second.
   */
  public static final int ROTATE_SPEED = 100;

  /**
   * Low speed used for more accurate sensor radians
   */
  public static final int LOW_SPEED = 15;

  /**
   * The motor acceleration in degrees per second squared.
   */
  public static final int ACCELERATION = 1000;

  /**
   * Timeout period in milliseconds.
   */
  public static final int TIMEOUT_PERIOD = 3000;

  /**
   * The tile size in centimeters.
   */
  public static final double TILE_SIZE = 30.48;

  /**
   * The furthest the ultrasonic distance can effectively see.
   */
  public static final int MAX_US_DISTANCE = 100;

  /**
   * The distance from the front of the ultrasonic sensor to the wheel base.
   */
  public static final int US_SENSOR_RADIUS = 5;

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
  public static final EV3LargeRegulatedMotor shooterMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

  /**
   * The LCD.
   */
  public static final TextLCD LCD = LocalEV3.get().getTextLCD();

  /**
   * The ultrasonic sensor.
   */
  // public static final EV3UltrasonicSensor US_SENSOR = new EV3UltrasonicSensor(SensorPort.S4);

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
  // public static final UltrasonicPoller usPoller = new UltrasonicPoller();

  /**
   * US localizer
   */
  // public static final UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer();

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

  /**
   * Test path finder TODO: the way the path finder is initialized is subject to change
   */
  public static final PathFinder pathFinder = PathFinder.test(15, 9, 4, 7, 6, 8, 0, 5, 4, 9, 6, 5, 15, 9, 12, 6);

  public static final Thread odoT = new Thread(odometer);
  public static final Thread cT = new Thread(colorPoller);
}
