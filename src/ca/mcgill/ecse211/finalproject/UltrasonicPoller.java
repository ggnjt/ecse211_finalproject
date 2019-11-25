package ca.mcgill.ecse211.finalproject;

import static ca.mcgill.ecse211.finalproject.Resources.US_SENSOR;
import static ca.mcgill.ecse211.finalproject.Resources.navigation;
import static ca.mcgill.ecse211.finalproject.Resources.odometer;

import java.util.Arrays;

import lejos.robotics.SampleProvider;

/**
 * A poller for the ultrasonic sensor. It runs continuously in its own thread,
 * polling the sensor about every 50 ms. After getting a value from the sensor,
 * it will convert the distance into centimeters and assign it to the distance
 * variable. This variable can be accessed by other classes by calling
 * getDistance().
 */
public class UltrasonicPoller implements Runnable {
	private int distance;
	private float[] usData;
	private static final short BUFFER_SIZE = 11;
	private int[] filterBuffer = new int[BUFFER_SIZE];
	private static boolean wait = false;
	private SampleProvider sampleProvider;

	public UltrasonicPoller() {
		usData = new float[US_SENSOR.sampleSize()];
		sampleProvider = US_SENSOR.getDistanceMode();
	}

	private static boolean obstacleDetected = false;

	/**
	 * run method for the US sensor
	 */
	public void run() {
		int reading;
		int count = 0;

		while (true) {
			if (wait) {
				try {
					Thread.sleep(50);
				} catch (Exception e) {
				} // Poor man's timed sampling
			} else {
				long readingStart, readingEnd;
				readingStart = System.currentTimeMillis();
				sampleProvider.fetchSample(usData, 0); // acquire distance data in meters
				reading = (int) (usData[0] * 100.0); // extract from buffer, convert to cm, cast to int
				// filling up the median filter and returning -1 as reading

				if (!Main.localizationFinished) {
					if (count < BUFFER_SIZE) {
						filterBuffer[count] = reading;
						distance = -1;
						count++;
					} else { // median filter
						shiftArray(filterBuffer, reading);
						int[] sample = filterBuffer.clone();
						Arrays.sort(sample); // cloning and sorting to preseve the buffer array
						distance = sample[BUFFER_SIZE / 2]; // reading median value
					}
				} else {
					distance = reading;
					obstacleDetected = (obstacleDetected
							|| (distance < Resources.ObstacleDetectionThreashold && distance > 0));
				}
				readingEnd = System.currentTimeMillis();
				if (readingEnd - readingStart < 70) {
					try {
						Thread.sleep(70 - (readingEnd - readingStart));
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				} else {
					try {
						Thread.sleep(20);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
			}
		}
		// Poor man's timed sampling
	}

	/**
	 * this method shifts the array by one position and enters a new integer ad the
	 * [0] position
	 * 
	 * @param arr  array
	 * @param newI new integer to be added
	 */
	void shiftArray(int[] arr, int newI) {
		int size = arr.length;
		for (int i = 0; i < size - 1; i++) {
			arr[i] = arr[i + 1];
		}
		arr[size - 1] = newI;
	}

	/**
	 * get the filtered distance reading
	 * 
	 * @return filtered reading of distance
	 */
	public int getDistance() {
		return this.distance;
	}

	public static void sleep() {
		wait = true;
	}

	public static void wake() {
		wait = false;
	}

	public static void resetDetection() {
		obstacleDetected = false;
	}

	public static boolean hasDetected() {
		return obstacleDetected;
	}
}
