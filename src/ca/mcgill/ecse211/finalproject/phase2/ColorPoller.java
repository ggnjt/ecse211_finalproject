package ca.mcgill.ecse211.finalproject.phase2;

import static ca.mcgill.ecse211.finalproject.Resources.*;

import lejos.robotics.SampleProvider;

/**
 * Class which takes care of color sensor polling, signal filtering as well as odometry correction using the color sensors
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
	/**
	 * a buffer used for the left sensor for line detection
	 */
	private static int leftCounter = 0;

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
	private static float rightDer = 0;
	/**
	 * used to hold previous value of the right sampler
	 */
	private static float rightPrev = 0;
	/**
	 * a buffer used for the right sensor for line detection
	 */
	private static int rightCounter = 0;

	/**
	 * A threshold value used to determine a large enough change in sensor
	 * measurement
	 */
	private static final float SENSOR_DERIVATIVE_THRESHOLD = 0.07f;

	/**
	 * a global kill switch for the poller thread
	 */
	public static boolean kill = false;
	/**
	 * whether or not the left sensor is currently detecting a black line
	 */
	private static boolean leftLineDetected;
	/**
	 * whether or not the right sensor is currently detecting a black line
	 */
	private static boolean rightLineDetected;
	/**
	 * flag indicating whether the robot is in the process of odometry correction
	 */
	public static boolean isCorrecting;
	/**
	 * run method for the two collor pollers
	 */
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
			if (leftLineDetected && rightLineDetected) {
				this.sleep();
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
		if (leftCounter > 2) {
			leftCounter = 0;
			return true;
		} else if (leftDer < 0 && Math.abs(leftDer) > SENSOR_DERIVATIVE_THRESHOLD) {
			leftCounter++;
			return false;
		} else
			return false;
	}

	/**
	 * Whether of not the right sensor is detecting a black line
	 * 
	 * @return boolean value indicating that the right sensor has transitioned onto
	 *         a black line
	 */
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

	/**
	 * corrects the odometer according to the nearest black line using internal
	 * value readings
	 */
	public static void correctXYT() {
		int updatedTheta;
		double[] currXYT = odometer.getXYT();
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
	
	public void resetLineDetection() {
		leftLineDetected = false;
		rightLineDetected = false;
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
	
	public void sleep() {
		try {
			this.wait();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
	
	public void sleepFor (long millis) {
		try {
			Thread.sleep(millis);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
	
	public void wake() {
		this.notify();
	}
	
	public boolean[] getLineDetectionStatus() {
		boolean [] result = {leftLineDetected, rightLineDetected};
		return result;
	}
	
	
}