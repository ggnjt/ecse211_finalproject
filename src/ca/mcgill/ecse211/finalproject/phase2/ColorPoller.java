package ca.mcgill.ecse211.finalproject.phase2;

import static ca.mcgill.ecse211.finalproject.Resources.TILE_SIZE;
import static ca.mcgill.ecse211.finalproject.Resources.leftColorSensor;
import static ca.mcgill.ecse211.finalproject.Resources.navigation;
import static ca.mcgill.ecse211.finalproject.Resources.odometer;
import static ca.mcgill.ecse211.finalproject.Resources.rightColorSensor;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

/**
 * Class which takes care of color sensor polling, signal filtering as well as
 * odometry correction using the color sensors
 * 
 * @author yp
 *
 */
public class ColorPoller implements Runnable {
	// =====left sensor=====//
	/**
	 * sample provider for the left color sensor
	 */
	private static SampleProvider leftSampleProvider = leftColorSensor.getRedMode();
	/**
	 * sample reading from the left color sensor
	 */
	private static float[] leftSampleColor = new float[leftColorSensor.sampleSize()];
	/**
	 * value from left sensor
	 */
	private static float leftSample;
	/**
	 * change in sensor measurement between two iterations of the left color sensor
	 * sampling
	 */
	private static float leftDer = 0;
	/**
	 * used to hold previous value of the left sampler
	 */
	private static float leftPrev = 0;

	// =====right sensor=====//
	/**
	 * sample provider for the right color sensor
	 */
	private static SampleProvider rightSampleProvider = rightColorSensor.getRedMode();
	/**
	 * sample reading from the right color sensor
	 */
	private static float[] rightSampleColor = new float[rightColorSensor.sampleSize()];
	/**
	 * value from right sensor
	 */
	private static float rightSample;
	/**
	 * change in sensor measurement between two iterations of the right color sensor
	 * sampling
	 */
	private static volatile float rightDer = 0f;
	/**
	 * used to hold previous value of the right sampler
	 */
	private static volatile float rightPrev = 0f;

	/**
	 * A threshold value used to determine a large enough change in sensor
	 * measurement
	 */
	private static final float SENSOR_DERIVATIVE_THRESHOLD = 0.1f;

	/**
	 * a global kill switch for the poller thread
	 */
	public static boolean kill = false;
	/**
	 * whether or not the left sensor is currently detecting a black line
	 */
	public static volatile boolean leftLineDetected;
	/**
	 * whether or not the right sensor is currently detecting a black line
	 */
	public static volatile boolean rightLineDetected;
	/**
	 * flag indicating whether the robot is in the process of odometry correction
	 */
	public static volatile boolean isCorrecting = false;

	public static boolean wait = false;

	/**
	 * run method for the two collor pollers
	 */
	public void run() {
		long readingStart, readingEnd;
		// int i = 0;
		while (!kill) {
			if (wait) {
				System.out.println("waiting");
				try {
					Thread.sleep(60);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			} else {
				// System.out.println(i);
				// i++;

				readingStart = System.currentTimeMillis();
				System.out.println("left:" + leftSample + "right: " + rightSample + " " + isCorrecting);
				leftSampleProvider.fetchSample(leftSampleColor, 0);
				rightSampleProvider.fetchSample(rightSampleColor, 0);
				leftSample = leftSampleColor[0];
				rightSample = rightSampleColor[0];
				leftDer = leftSample - leftPrev;
				rightDer = rightSample - rightPrev;
				if (!leftLineDetected) { // If we have already detected a line, don't try to detect it again because it
											// will give
											// a false negative
					leftLineDetected = leftDetectBlackLine();
					isCorrecting = !isCorrecting && rightLineDetected;
					if (isCorrecting && leftLineDetected) {
						navigation.stopTheRobot();
						navigation.setSpeed(40);
						synchronized (navigation) {
							navigation.notify();
						}
					}
				}

				if (!rightLineDetected) { // If we have already detected a line, don't try to detect it again because it
											// will give
											// a false negative
					rightLineDetected = rightDetectBlackLine();
					isCorrecting = !isCorrecting && leftLineDetected;
					if (isCorrecting && rightLineDetected) {
						navigation.stopTheRobot();
						navigation.setSpeed(40);
						synchronized (navigation) {
							navigation.notify();
						}
					}
				}
				leftPrev = leftSample;
				rightPrev = rightSample;
				readingEnd = System.currentTimeMillis();
				if (readingEnd - readingStart < 75) {
					try {
						Thread.sleep(75 - (readingEnd - readingStart));
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
			}
		}
	}

	/**
	 * get the reading from the left color sensor
	 * 
	 * @return reading of red light from left sensor
	 */
	public static float getLeftSample() {
		return leftSample;
	}

	/**
	 * get the reading from the right color sensor
	 * 
	 * @return reading of red light from right sensor
	 */
	public static float getRightSample() {
		return rightSample;
	}

	/**
	 * Whether of not the left sensor is detecting a black line
	 * 
	 * @return boolean value indicating that the left sensor has transitioned onto a
	 *         black line
	 */
	public static boolean leftDetectBlackLine() {
		if (leftDer < 0 && Math.abs(leftDer) > SENSOR_DERIVATIVE_THRESHOLD) {
			Sound.beep();
			return true;
		} else {
			return false;
		}

	}

	/**
	 * Whether of not the right sensor is detecting a black line
	 * 
	 * @return boolean value indicating that the right sensor has transitioned onto
	 *         a black line
	 */
	public static boolean rightDetectBlackLine() {
		if (rightDer < 0 && Math.abs(rightDer) > SENSOR_DERIVATIVE_THRESHOLD) {
			Sound.twoBeeps();
			return true;
		} else {
			return false;
		}
	}

	/**
	 * corrects the odometer according to the nearest black line using internal
	 * value readings
	 */
	public static void correctXYT() {
		double[] currXYT = odometer.getXYT();
		/*
		 * Resets the angle (since we are only driving in cardinal directions, there are
		 * only 4 possibilities
		 */
		switch (navigation.orientation) {
		case NORTH:
			odometer.setTheta(0);
			correctY(currXYT);
			navigation.yTile++;
			break;
		case SOUTH:
			odometer.setTheta(180);
			correctY(currXYT);
			navigation.yTile--;
			break;
		case EAST:
			odometer.setTheta(90);
			correctX(currXYT);
			navigation.xTile++;
			break;
		case WEST:
			odometer.setTheta(270);
			correctX(currXYT);
			navigation.xTile--;
			break;
		default:
			break;
		}
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
	private static void correctX(double[] currXYT) {
		double currX = currXYT[0];
		currX = TILE_SIZE * (Math.round((currX) / TILE_SIZE));
		odometer.setX(currX);
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
	private static void correctY(double[] currXYT) {
		double currY = currXYT[1];
		currY = TILE_SIZE * (Math.round((currY) / TILE_SIZE));
		odometer.setY(currY);
	}

	public static void resetLineDetection() {
		isCorrecting = false;
		leftLineDetected = false;
		rightLineDetected = false;

		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
		}
	}

	public void sleep() {
		wait = true;
	}

	public void wake() {
		wait = false;
	}

	public boolean[] getLineDetectionStatus() {
		System.out.println("left= " + leftLineDetected + " right= " + rightLineDetected);
		boolean[] result = { leftLineDetected, rightLineDetected };
		return result;
	}

}
