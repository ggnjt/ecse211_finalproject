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
					Thread.sleep(100);
				} catch (Exception e) {
				} // Poor man's timed sampling
			} else {
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
					boolean facingAWall = false;

					double[] currXYT = odometer.getXYT();
					if (currXYT[2] >= 45 && currXYT[2] < 135) {// facing EAST
						if (navigation.xTile + 1 > Resources.ARENA_X - 1) {
							facingAWall = true;
						}
					} else if (currXYT[2] >= 135 && currXYT[2] < 225) {// facing SOUTH
						if (navigation.yTile - 1 < 0) {
							facingAWall = true;
						}
					} else if (currXYT[2] >= 225 && currXYT[2] < 315) {// facing WEST
						if (navigation.xTile - 1 < 0) {
							facingAWall = true;
						}
					} else {
						if (navigation.yTile + 1 > Resources.ARENA_Y - 1) {
							facingAWall = true;
						}
					}

					if (facingAWall) {
						System.out.println("ZOMFG!!!");
					}
					obstacleDetected = !facingAWall
							&& (obstacleDetected || (distance < Resources.ObstacleDetectionThreashold && distance > 0));
				}
				try {
					Thread.sleep(50);
				} catch (Exception e) {
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
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public static boolean hasDetected() {
		return obstacleDetected;
	}
}
