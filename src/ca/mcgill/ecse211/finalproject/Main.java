package ca.mcgill.ecse211.finalproject;

import static ca.mcgill.ecse211.finalproject.Resources.TEAM_NUMBER;
import static ca.mcgill.ecse211.finalproject.Resources.colorPoller;
import static ca.mcgill.ecse211.finalproject.Resources.navigation;
import static ca.mcgill.ecse211.finalproject.Resources.odometer;
import static ca.mcgill.ecse211.finalproject.Resources.redTeam;

import java.util.ArrayList;
import java.util.Arrays;

import ca.mcgill.ecse211.finalproject.Navigation.TravelingMode;
import ca.mcgill.ecse211.finalproject.phase2.ColorSampler;
import ca.mcgill.ecse211.finalproject.phase2.PathFinder;
//import ca.mcgill.ecse211.finalproject.phase1.UltrasonicLocalizer;
import lejos.hardware.Button;
import lejos.hardware.Sound;

/**
 * The main driver class for the odometry lab.
 */
public class Main {
	public static boolean P1finished = false;

	/**
	 * The main entry point.
	 * 
	 * @param args
	 */
	public static void main(String[] args) {

//		Thread USPollerThread = new Thread(Resources.usPoller);
//		Thread USLocalizerThread = new Thread(Resources.usLocalizer);
		odometer.start();
//		USPollerThread.start();
//		USLocalizerThread.start();
//		while (!P1finished) {
//			Main.sleepFor(1000);
//		}
//		Sound.beepSequence();
//
//		UltrasonicPoller.sleep();// this should be removed after demo
//
//		try {
//			USPollerThread.join(5000); // this should be removed after demo
//			USLocalizerThread.join(5000);
//		} catch (InterruptedException e) {
//			System.out.println("Sadness is the ichor of life");
//		}

		Resources.pathFinder = new PathFinder(redTeam == TEAM_NUMBER);
		
		targetFinderTest();
		
		Resources.leftMotor.setStallThreshold(20, 10);
		Resources.rightMotor.setStallThreshold(20, 10);
		
		Thread cT = new Thread(colorPoller);
		cT.start();

		//stressTest();
		// ArrayList<int[]> moves = Resources.pathFinder.findPath();

//		for (int[] move : moves) {
//			System.out.println(Arrays.toString(move));
//			navigation.setSpeed(Resources.LOW_FORWARD_SPEED);
//
//			colorPoller.sleep();
//			ColorSampler.sleep();
//			Main.sleepFor(60);
//			System.gc();
//			colorPoller.wake();
//			ColorSampler.wake();
//
//			navigation.processNextMove(move);
//
//			while (!navigation.moveSuccessful || Navigation.interrupted) {
//				if (navigation.navigationMode == TravelingMode.TRAVELING) {
//					navigation.processNextMove(move);
//				} else {
//					Main.sleepFor(60);
//				}
//			}
//		}
//		colorPoller.sleep();
//		navigation.goToLowerLeftCorner();
//		navigation.turnTo((Resources.targetAngle + 180) % 360);
//		Sound.beep();
//		Main.sleepFor(500);
//		Sound.beep();
//		Main.sleepFor(500);
//		Sound.beep();
//		Resources.shooterMotor.rotate(165);
//		Button.waitForAnyPress();
//
//		System.exit(0);
	}

	public static void sleepFor(long duration) {
		try {
			Thread.sleep(duration);
		} catch (InterruptedException e) {
			// There is nothing to be done here
		}
	}

	public static void targetFinderTest() {
		System.out.println("~~~~~~~~Initial target~~~~~~~~");
		Resources.pathFinder.printMap();
		Resources.pathFinder.setObstacle(PathFinder.targetX, PathFinder.targetY);
		Resources.pathFinder.findPath();
		System.out.println("~~~~~~~~~~New target~~~~~~~~~~");
		Resources.pathFinder.printMap();
		System.exit(0);
	}
	
	public static void stressTest() {
		ArrayList<int[]> moves = new ArrayList<int[]>();
		moves.add(new int[] { 0, 1 });
		moves.add(new int[] { 0, 2 });
		moves.add(new int[] { 0, 3 });
		moves.add(new int[] { 0, 4 });
		moves.add(new int[] { 1, 4 });
		moves.add(new int[] { 1, 3 });
		moves.add(new int[] { 1, 2 });
		moves.add(new int[] { 1, 1 });
		moves.add(new int[] { 1, 0 });
		moves.add(new int[] { 2, 0 });
		moves.add(new int[] { 2, 1 });
		moves.add(new int[] { 2, 2 });
		moves.add(new int[] { 2, 3 });
		moves.add(new int[] { 2, 4 });
		moves.add(new int[] { 3, 4 });
		moves.add(new int[] { 3, 3 });
		moves.add(new int[] { 3, 2 });
		moves.add(new int[] { 3, 1 });
		moves.add(new int[] { 3, 0 });
		moves.add(new int[] { 4, 0 });
		moves.add(new int[] { 4, 1 });
		moves.add(new int[] { 4, 2 });
		moves.add(new int[] { 4, 3 });
		moves.add(new int[] { 4, 4 });
		moves.add(new int[] { 4, 3 });
		moves.add(new int[] { 4, 2 });
		moves.add(new int[] { 4, 1 });
		moves.add(new int[] { 4, 0 });
		moves.add(new int[] { 3, 0 });
		moves.add(new int[] { 2, 0 });
		moves.add(new int[] { 1, 0 });
		moves.add(new int[] { 0, 0 });

		while (true) {
			for (int[] move : moves) {
				System.out.println(Arrays.toString(move));
				
				navigation.setSpeed(Resources.LOW_FORWARD_SPEED);
				
				navigation.processNextMove(move);

				while (!navigation.moveSuccessful || Navigation.interrupted) {
					if (navigation.navigationMode == TravelingMode.TRAVELING) {
						navigation.processNextMove(move);
						Main.sleepFor(20);
					} else {
						Main.sleepFor(70);
					}
				}
			}
		}

	}
}
