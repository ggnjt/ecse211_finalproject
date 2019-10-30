//package ca.mcgill.ecse211.finalproject;
//
//import static ca.mcgill.ecse211.finalproject.Resources.*;
//
//import lejos.hardware.Sound;
//import lejos.robotics.SampleProvider;
//
//public class ColorPoller implements Runnable {
//	// timer
//	//private long leftTimer;
//	//private long rightTimer;
//
//	// left sensor
//	private static SampleProvider leftSampleProvider;
//	private static float[] leftSampleColor;
//	private static float leftSample; // value from sensor
//	private static float leftDer = 0; // change in sensor measurement
//	private static float leftPrev = 0; // used to hold previous value
//	private static int leftCounter = 0; // used as a buffer for signal change
//	// right sensor
//	private static SampleProvider rightSampleProvider;
//	private static float[] rightSampleColor;
//	private static float rightSample; // value from sensor
//	private static float rightDer = 0; // change in sensor measurement
//	private static float rightPrev = 0; // used to hold previous value
//	private static int rightCounter = 0; // used as a buffer for signal change
//
//	private static final float SENSOR_DERIVATIVE_THRESHOLD = 0.03f;
//	public static boolean kill = false;
//
//	public ColorPoller() {
//		leftSampleProvider = leftColorSensor.getRedMode();
//		leftSampleColor = new float[leftColorSensor.sampleSize()];
//		rightSampleProvider = rightColorSensor.getRedMode();
//		rightSampleColor = new float[rightColorSensor.sampleSize()];
//	}
//	
//	public void run() {
//		long readingStart, readingEnd;
//
//		while (!kill) {
//			readingStart = System.currentTimeMillis();
//			
//			leftSampleProvider.fetchSample(leftSampleColor, 0);
//			rightSampleProvider.fetchSample(rightSampleColor, 0);
//			leftSample = leftSampleColor[0];
//			rightSample = rightSampleColor[0];
//			leftDer = leftSample - leftPrev;
//			rightDer = rightSample - rightPrev;
//			leftPrev = leftSample;
//			rightPrev = rightSample;
//			
//			System.out.println("left: " + leftSample);
//			System.out.println("right: " + rightSample);
//			
//			readingEnd = System.currentTimeMillis();
//			if (readingEnd - readingStart < 100) {
//				try {
//					Thread.sleep(100 - (readingEnd - readingStart));
//				} catch (InterruptedException e) {
//					// TODO Auto-generated catch block
//					e.printStackTrace();
//				}
//			}
//		}
//	}
//
//	public static float getLeftSample() {
//		return leftSample;
//	}
//
//	public static float getRightSample() {
//		return rightSample;
//	}
//
//	public static boolean leftDetectBlackLine() {
//	
//		
//		if (leftCounter > 2) {
//			leftCounter = 0;
//			Sound.beepSequenceUp();
//			return true;
//		} else if (leftDer < 0 && Math.abs(leftDer) > SENSOR_DERIVATIVE_THRESHOLD) {
//			leftCounter++;
//			return false;
//		} else
//			return false;
//	}
//
//	public static boolean rightDetectBlackLine() {
//		
//		if (rightCounter > 2) {
//			rightCounter = 0;
//			Sound.beepSequence();
//			return true;
//		} else if (rightDer < 0 && Math.abs(rightDer) > SENSOR_DERIVATIVE_THRESHOLD) {
//			rightCounter++;
//			return false;
//		} else
//			return false;
//	}
//}
