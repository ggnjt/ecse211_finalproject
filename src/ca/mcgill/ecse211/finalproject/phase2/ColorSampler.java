package ca.mcgill.ecse211.finalproject.phase2;

import ca.mcgill.ecse211.finalproject.Resources;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

/**
 * Color Sampler
 * 
 * @author elias
 *
 */
public class ColorSampler implements Runnable {
	/**
	 * SampleProvider
	 */
	private SampleProvider sampleProvider;

	/**
	 * sample reading from the color sensor
	 */
	private float[] sample;

	/**
	 * private float prev
	 */
	private float prev = 0.0f;

	/**
	 * private float der
	 */
	private float der = 0.0f;

	/**
	 * blackLine
	 */
	private boolean blackLine = false;
	
	/**
	 * current reading
	 */
	public float currentSample = 0.0f;
	
	public static boolean wait;

	/**
	 * Consturctor from a color sensor
	 * 
	 * @param colorSensor
	 */
	public ColorSampler(EV3ColorSensor colorSensor) {
		super();
		this.sampleProvider = colorSensor.getRedMode();
		;
		this.sample = new float[colorSensor.sampleSize()];
	}

	/**
	 * run method for the two color poller
	 */
	public void run() {
		long readingStart, readingEnd;

		while (true) {
			if (wait) {
				try {
					Thread.sleep(60);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			} else {
				readingStart = System.currentTimeMillis();
				sampleProvider.fetchSample(sample, 0);
				currentSample = sample[0];
				der = currentSample - prev;
				blackLine = (der < 0 && Math.abs(der) > Resources.INTENSITY_THRESHOLD);
				readingEnd = System.currentTimeMillis();
				prev = currentSample;

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
	}

	/**
	 * get derivative
	 */
	public float getDer() {
		return der;
	}

	public boolean getBlackLine() {
		return blackLine;
	}
	
	public static void sleep() {
		wait = true;
	}
	
	public static void wake () {
		wait = false;
	}
}
