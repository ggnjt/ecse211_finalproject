package ca.mcgill.ecse211.finalproject.phase2;

import static ca.mcgill.ecse211.finalproject.Resources.TILE_SIZE;
import static ca.mcgill.ecse211.finalproject.Resources.leftColorSensor;
import static ca.mcgill.ecse211.finalproject.Resources.leftMotor;
import static ca.mcgill.ecse211.finalproject.Resources.odometer;
import static ca.mcgill.ecse211.finalproject.Resources.rightColorSensor;
import static ca.mcgill.ecse211.finalproject.Resources.rightMotor;
import ca.mcgill.ecse211.finalproject.Main;
import ca.mcgill.ecse211.finalproject.Navigation;
import ca.mcgill.ecse211.finalproject.Navigation.TravelingMode;
import ca.mcgill.ecse211.finalproject.Resources;
import ca.mcgill.ecse211.finalproject.UltrasonicPoller;

/**
 * Class which takes care of color sensor polling, signal filtering as well as
 * odometry correction using the color sensors
 * 
 * @author yp
 * @author holmsi
 *
 */
public class ColorPoller implements Runnable {

	/**
	 * whether or not the left sensor is currently detecting a black line
	 */
	private static boolean leftLineDetected = false;
	/**
	 * whether or not the right sensor is currently detecting a black line
	 */
	private static boolean rightLineDetected = false;

	/**
	 * boolean to make the thread wait while turning
	 */
	private static boolean wait = false;

	/**
	 * left light Sampler
	 */
	public static ColorSampler leftSampler;

	/**
	 * right light Sampler
	 */
	public static ColorSampler rightSampler;

	/**
	 * leftColorThread
	 */
	private static Thread leftColorThread;

	/**
	 * RightColorThread
	 */
	private static Thread rightColorThread;

	/**
	 * safety net counter for no line detection on the right
	 */
	private static int rightCounter = 0;

	/**
	 * safety net counter for no line detection on the left
	 */
	private static int leftCounter = 0;

	/**
	 * constructor, starts both left and right threads
	 */
	public ColorPoller() {
		super();
		leftSampler = new ColorSampler(leftColorSensor);
		rightSampler = new ColorSampler(rightColorSensor);

		leftColorThread = new Thread(leftSampler);
		rightColorThread = new Thread(rightSampler);

		leftColorThread.start();
		rightColorThread.start();
	}

	/**
	 * run method for the two color poller
	 */
	@Override
	public void run() {
		while (true) {
			// sleep
			if (wait) {
				try {
					Thread.sleep(50);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			} else {
				long readingStart, readingEnd;
				readingStart = System.currentTimeMillis();
				switch (Navigation.navigationMode) {
				case TRAVELING:
					// continuously detect line
					leftLineDetected = leftSampler.getBlackLine();
					rightLineDetected = rightSampler.getBlackLine();
					if (leftLineDetected || rightLineDetected) {
						// line detection during traveling, state change and slow the robot down
						Navigation.stopTheRobot();
						Main.sleepFor(50);
						Navigation.navigationMode = TravelingMode.CORRECTING;
						Navigation.setSpeed(Resources.CORRECTION_SPPED);
						Navigation.moveSuccessful = false;
						Navigation.interrupted = true;
					} else {
						rightCounter = 0;
						leftCounter = 0;
					}
					break;
				case OBSTACLE_ENCOUNTERED:
					// not sure if anything should be done when encountered an obstacle, but either
					// way it should not detect anything since the detection only happens within
					// the middle of the square
					break;
				case CORRECTING:
					// probe for the other line detection
					leftLineDetected = leftLineDetected || leftSampler.getBlackLine();
					rightLineDetected = rightLineDetected || rightSampler.getBlackLine();
					boolean stopped = !leftMotor.isMoving() && !rightMotor.isMoving();

					// first fail safe: if both lines present similar reading then they both are on
					// the line
					if (leftLineDetected && rightLineDetected
							|| Math.abs(leftSampler.currentSample - rightSampler.currentSample) < 0.09) {
						// tweak the number to get better line detection
						// Correct
						correctXYT();
						synchronized (Resources.leftMotor) {
							synchronized (Resources.rightMotor) {
								Navigation.stopTheRobot();
								Navigation.setSpeed(Resources.HIGH_FORWARD_SPEED);
								Navigation.navigationMode = TravelingMode.TRAVELING;
								UltrasonicPoller.resetDetection();
							}
						}
						resetLineDetection();
					} else if (leftLineDetected) {
						if (stopped) {
							synchronized (rightMotor) {
								synchronized (leftMotor) {
									rightMotor.forward();
								}
							}
						}
						rightCounter++;
						// second fail safe: if miss line reading move this many cycles
						// tweak the number if the robot turns too much if it does not detect a black
						// line or does not correct enough for some reason
						if (rightCounter > 20) {
							synchronized (leftMotor) {
								synchronized (rightMotor) {
									Navigation.stopTheRobot();
								}
							}
							rightLineDetected = true;
							rightCounter = 0;
						}
					} else if (rightLineDetected) {
						if (stopped) {
							synchronized (leftMotor) {
								synchronized (rightMotor) {
									leftMotor.forward();
								}
							}
						}
						leftCounter++;
						if (leftCounter > 20) {
							synchronized (leftMotor) {
								synchronized (rightMotor) {
									Navigation.stopTheRobot();
								}
							}
							leftLineDetected = true;
							leftCounter = 0;
						}
					}
					break;
				}

				readingEnd = System.currentTimeMillis();
				if (readingEnd - readingStart < 70) {
					try {
						Thread.sleep(70 - (readingEnd - readingStart));
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				} else {
					// always sleep because sleep deprivation make threads unhappy
					try {
						Thread.sleep(20);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
			}
		}
	}

	/**
	 * corrects the odometer according to the nearest black line using internal
	 * value readings
	 */
	public static void correctXYT() {
		double updatedTheta = 0d;
		double[] currXYT = odometer.getXYT();
		/*
		 * Resets the angle (since we are only driving in cardinal directions, there are
		 * only 4 possibilities
		 */
		if (currXYT[2] >= 45 && currXYT[2] < 135) {
			updatedTheta = 90d; // facing EAST
			correctX(currXYT[0], updatedTheta);
		} else if (currXYT[2] >= 135 && currXYT[2] < 225) {
			updatedTheta = 180d; // facing SOUTH
			correctY(currXYT[1], updatedTheta);
		} else if (currXYT[2] >= 225 && currXYT[2] < 315) {
			updatedTheta = 270d; // facing WEST
			correctX(currXYT[0], updatedTheta);
		} else {
			updatedTheta = 0d; // facing NORTH
			correctY(currXYT[1], updatedTheta);
		}
		odometer.setTheta(updatedTheta);
	}

	/**
	 *
	 * Reset the X position based on what tile the robot thinks it is on, if the
	 * robot is less than half a tile away from its current position in each
	 * direction, correction is possible
	 *
	 * @param currXYT
	 * @param updatedTheta
	 */
	private static void correctX(double currX, double updatedTheta) {
		currX = TILE_SIZE * (Math.round((currX) / TILE_SIZE));
		if (updatedTheta == 90d) {
			odometer.setX(currX - Resources.SENSOR_RADIUS);
		} else {
			odometer.setX(currX + Resources.SENSOR_RADIUS);
		}
	}

	/**
	 *
	 * Reset the Y position based on what tile the robot thinks it is on, if the
	 * robot is less than half a tile away from its current position in each
	 * direction, correction is possible
	 * 
	 * @param currXYT
	 * @param updatedTheta
	 */
	private static void correctY(double currY, double updatedTheta) {
		currY = TILE_SIZE * (Math.round((currY) / TILE_SIZE));
		if (updatedTheta == 0d) {
			odometer.setY(currY - Resources.SENSOR_RADIUS);
		} else {
			odometer.setY(currY + Resources.SENSOR_RADIUS);
		}
	}

	/**
	 * reset the line detect booleans to false
	 */
	public static void resetLineDetection() {
		leftLineDetected = false;
		rightLineDetected = false;
		try {
			Thread.sleep(2400);
		} catch (InterruptedException e) {
		}
	}

	/**
	 * makes the color poller thread sleep while turning
	 */
	public static void sleep() {
		wait = true;
	}

	/**
	 * wake the thread up after sleeping
	 */
	public static void wake() {
		try {
			Thread.sleep(50);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		wait = false;
	}
}
