package ca.mcgill.ecse211.finalproject;

import static ca.mcgill.ecse211.finalproject.Resources.FORWARD_SPEED;
import static ca.mcgill.ecse211.finalproject.Resources.leftColorSensor;
import static ca.mcgill.ecse211.finalproject.Resources.leftMotor;
import static ca.mcgill.ecse211.finalproject.Resources.rightColorSensor;
import static ca.mcgill.ecse211.finalproject.Resources.rightMotor;
import static ca.mcgill.ecse211.finalproject.Resources.TILE_SIZE;
import static ca.mcgill.ecse211.finalproject.Resources.odometer;
import lejos.robotics.SampleProvider;

public class ColorPoller implements Runnable {
	// timer
	private long leftTimer;
	private long rightTimer;

	// left sensor
	private static SampleProvider leftSampleProvider = leftColorSensor.getRedMode();
	private static float[] leftSampleColor = new float[leftColorSensor.sampleSize()];
	private static float leftSample; // value from sensor
	private static float leftDer = 0; // change in sensor measurement
	private static float leftPrev = 0; // used to hold previous value
	private static int leftCounter = 0; // used to hold previous value
	// right sensor
	private static SampleProvider rightSampleProvider = rightColorSensor.getRedMode();
	private static float[] rightSampleColor = new float[rightColorSensor.sampleSize()];
	private static float rightSample; // value from sensor
	private static float rightDer = 0; // change in sensor measurement
	private static float rightPrev = 0; // used to hold previous value
	private static int rightCounter = 0; // used to hold previous value

	private static final float SENSOR_DERIVATIVE_THRESHOLD = 0.03f;
	public static boolean kill = false;
	private static boolean leftLineDetected;
	private static boolean rightLineDetected;

	private static double[] currXYT;

	public void run() {
		long readingStart, readingEnd;

		while (!kill) {
			readingStart = System.currentTimeMillis();

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
			}
			if (!rightLineDetected) { // If we have already detected a line, don't try to detect it again because it
										// will give
										// a false negative
				rightLineDetected = rightDetectBlackLine();
			}

			if ((leftLineDetected && rightLineDetected) || !(leftLineDetected && rightLineDetected)) { // Both lines detected, (or no detections), carry on
				leftMotor.setSpeed(FORWARD_SPEED);
				rightMotor.setSpeed(FORWARD_SPEED);
				leftLineDetected = false;
				rightLineDetected = false;
			} else if (leftLineDetected && !rightLineDetected) { // Left side detection, stop left motor keep right
																	// running
				leftMotor.stop(true);
			} else if (rightLineDetected && !leftLineDetected) { // Right side detection, stop right motor keep left
																	// running
				rightMotor.stop(true);
			}

			leftPrev = leftSample;
			rightPrev = rightSample;

			readingEnd = System.currentTimeMillis();
			if (readingEnd - readingStart < 100) {
				try {
					Thread.sleep(100 - (readingEnd - readingStart));
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}
	}

	public static float getLeftSample() {
		return leftSample;
	}

	public static float getRightSample() {
		return rightSample;
	}

	public static boolean leftDetectBlackLine() {
		if (leftCounter > 2) {
			leftCounter = 0;
			return true;
		} else if (leftDer < 0 && Math.abs(leftDer) > SENSOR_DERIVATIVE_THRESHOLD) {
			leftCounter++;
			return false;
		} else
			return false;
	}

	public static boolean rightDetectBlackLine() {
		if (rightCounter > 2) {
			rightCounter = 0;
			return true;
		} else if (rightDer < 0 && Math.abs(rightDer) > SENSOR_DERIVATIVE_THRESHOLD) {
			rightCounter++;
			return false;
		} else
			return false;
	}

	private static void correctXYT() { // Set theta to be on a cardinal direction
		int updatedTheta;
		currXYT = odometer.getXYT();
		/*
		 * Resets the angle (since we are only driving in cardinal directions, there are
		 * only 4 possibilities
		 */
		if (currXYT[2] >= 315 || currXYT[2] < 45) {
			updatedTheta = 0;
			odometer.setTheta(updatedTheta);
			correctY(currXYT, updatedTheta);
		} else if (currXYT[2] >= 45 && currXYT[2] < 135) {
			updatedTheta = 90;
			odometer.setTheta(updatedTheta);
			correctX(currXYT, updatedTheta);
		} else if (currXYT[2] > 135 && currXYT[2] < 225) {
			updatedTheta = 180;
			odometer.setTheta(updatedTheta);
			correctY(currXYT, updatedTheta);
		} else if (currXYT[2] >= 225 && currXYT[2] < 315) {
			updatedTheta = 270;
			odometer.setTheta(updatedTheta);
			correctX(currXYT, updatedTheta);
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
	private static void correctX(double[] currXYT, int updatedTheta) {
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
	private static void correctY(double[] currXYT, int updatedTheta) {
		double currY = currXYT[1];
		currY = TILE_SIZE * (Math.round((currY) / TILE_SIZE));
		odometer.setY(currY);
	}
}
