package ca.mcgill.ecse211.finalproject;

/**
 * thread for obstacle detection
 * 
 * @author alhomsi
 *
 */
public class UltrasonicObstacleDetector implements Runnable {
	/**
	 * buffer for counting derivative jumps
	 */
	private static int spaceCounter = 0;
	/**
	 * place holder to store the reading from the sensor
	 */
	private static int reading;
	/**
	 * global kill switch for the thread
	 */
	public static boolean kill = false;
	/**
	 * whether the US senbsor detects an obstacle or not
	 */
	public static boolean obstacleDetected = false;

	/**
	 * run method for the thread
	 */
	@Override
	public void run() {
		while (!kill) {
			reading = Resources.usPoller.getDistance();
			if (reading == -1)
				continue;

			if (reading < Resources.TILE_SIZE && spaceCounter < 3) {
				spaceCounter++;
			} else if (spaceCounter == 3) {
				obstacleDetected = true;
				spaceCounter = 0;
			}
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}
}
