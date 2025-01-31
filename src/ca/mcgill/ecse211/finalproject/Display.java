package ca.mcgill.ecse211.finalproject;

import static ca.mcgill.ecse211.finalproject.Resources.LCD;
import static ca.mcgill.ecse211.finalproject.Resources.navigation;
import static ca.mcgill.ecse211.finalproject.Resources.odometer;
import java.text.DecimalFormat;

/**
 * This class is used to display the content of the odometer variables (x, y,
 * Theta)
 */
public class Display implements Runnable {

	/**
	 * XYT values of the odometer
	 */
	private double[] position;
	/**
	 * update period of the display
	 */
	private final long DISPLAY_PERIOD = 500;
	/**
	 * max timeout value of the display
	 */
	private long timeout = Long.MAX_VALUE;

	public void run() {
		LCD.clear();

		long updateStart, updateEnd;

		long tStart = System.currentTimeMillis();
		do {
			updateStart = System.currentTimeMillis();
			LCD.drawString(navigation.navigationMode.toString(), 0, 0);
			// Retrieve x, y and Theta information
			position = odometer.getXYT();

			DecimalFormat numberFormat = new DecimalFormat("######0.00");

			LCD.drawString("X: " + numberFormat.format(position[0]), 0, 1);
			LCD.drawString("Y: " + numberFormat.format(position[1]), 0, 2);
			LCD.drawString("T: " + numberFormat.format(position[2]), 0, 3);

			// this ensures that the data is updated only once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < DISPLAY_PERIOD) {
				try {
					Thread.sleep(DISPLAY_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		} while ((updateEnd - tStart) <= timeout);
	}

	/**
	 * Sets the timeout in ms.
	 * 
	 * @param timeout
	 */
	public void setTimeout(long timeout) {
		this.timeout = timeout;
	}

	/**
	 * Shows the text on the LCD, line by line.
	 * 
	 * @param strings comma-separated list of strings, one per line
	 */
	public static void showText(String... strings) {
		LCD.clear();
		for (int i = 0; i < strings.length; i++) {
			LCD.drawString(strings[i], 0, i);
		}
	}

}
