package ca.mcgill.ecse211.finalproject.phase2;

import static ca.mcgill.ecse211.finalproject.Resources.FORWARD_SPEED;
import static ca.mcgill.ecse211.finalproject.Resources.TILE_SIZE;
import static ca.mcgill.ecse211.finalproject.Resources.leftColorSensor;
import static ca.mcgill.ecse211.finalproject.Resources.leftMotor;
import static ca.mcgill.ecse211.finalproject.Resources.navigation;
import static ca.mcgill.ecse211.finalproject.Resources.odometer;
import static ca.mcgill.ecse211.finalproject.Resources.rightColorSensor;
import static ca.mcgill.ecse211.finalproject.Resources.rightMotor;

import java.util.Arrays;

import ca.mcgill.ecse211.finalproject.Navigation.TravelingMode;
import ca.mcgill.ecse211.finalproject.Resources;
import lejos.hardware.Sound;

/**
 * Class which takes care of color sensor polling, signal filtering as well as
 * odometry correction using the color sensors
 * 
 * @author yp
 *
 */
public class ColorPoller implements Runnable {

	/**
	 * whether or not the left sensor is currently detecting a black line
	 */
	private boolean leftLineDetected = false;
	/**
	 * whether or not the right sensor is currently detecting a black line
	 */
	private boolean rightLineDetected = false;

	/**
	 * boolean to make the thread wait while turning
	 */
	public static boolean wait = false;

	/**
	 * left light Sampler
	 */
	ColorSampler leftSampler;

	/**
	 * right light Sampler
	 */
	ColorSampler rightSampler;

	/**
	 * leftColorThread
	 */
	Thread leftColorThread;

	/**
	 * RightColorThread
	 */
	Thread rightColorThread;

	/**
	 * safety net counter for no line detection on the right
	 */
	private int rightCounter = 0;

	/**
	 * safety net counter for no line detection on the left
	 */
	private int leftCounter = 0;

	/**
	 * constructor
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
			if (wait) {
				try {
					Thread.sleep(30);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			} else {
				long readingStart, readingEnd;
				readingStart = System.currentTimeMillis();
				switch (navigation.navigationMode) {
				case TRAVELING:
					leftLineDetected = leftSampler.getBlackLine();
					rightLineDetected = rightSampler.getBlackLine();
					if (leftLineDetected || rightLineDetected) {
						navigation.stopTheRobot();
						navigation.navigationMode = TravelingMode.CORRECTING;
						navigation.setSpeed(Resources.CORRECTION_SPPED);
						navigation.moveSuccessful = false;
					} else {
						rightCounter = 0;
						leftCounter = 0;
					}
					break;
				case OBSTACLE_ENCOUNTERED:
					break;
				case CORRECTING:
					leftLineDetected = leftLineDetected || leftSampler.getBlackLine();
					rightLineDetected = rightLineDetected || rightSampler.getBlackLine();
					boolean stopped = !leftMotor.isMoving() && !rightMotor.isMoving();
					if (leftLineDetected && rightLineDetected
							|| Math.abs(leftSampler.prev - rightSampler.prev) < 0.02) {
						// Correct
						correctXYT();
						// clear the line
						leftMotor.rotate(60, true);
						rightMotor.rotate(60, false);
						navigation.stopTheRobot();
						navigation.setSpeed(FORWARD_SPEED);
						navigation.navigationMode = TravelingMode.TRAVELING;
						resetLineDetection();
					} else if (leftLineDetected) {
						if (stopped) {
							rightMotor.forward();
						}
						rightCounter++;
						if (rightCounter > 40) { // fail safe
							navigation.stopTheRobot();
							rightLineDetected = true;
							rightCounter = 0;
						}
					} else if (rightLineDetected) {
						if (stopped) {
							leftMotor.forward();
						}
						leftCounter++;
						if (leftCounter > 40) {
							navigation.stopTheRobot();
							leftLineDetected = true;
							leftCounter = 0;
						}
					}
					break;
				}

				readingEnd = System.currentTimeMillis();
				if (readingEnd - readingStart < 50) {
					try {
						Thread.sleep(50 - (readingEnd - readingStart));
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
	public void resetLineDetection() {
		leftLineDetected = false;
		rightLineDetected = false;

		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
		}
	}

	/**
	 * makes the color poller thread sleep while turning
	 */
	public void sleep() {
		wait = true;
	}

	/**
	 * wake the thread up after sleeping
	 */
	public void wake() {
		wait = false;
	}
}
