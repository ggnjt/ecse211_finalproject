package ca.mcgill.ecse211.finalproject;

import static ca.mcgill.ecse211.finalproject.Resources.TEAM_NUMBER;
import static ca.mcgill.ecse211.finalproject.Resources.colorPoller;
import static ca.mcgill.ecse211.finalproject.Resources.odometer;
import static ca.mcgill.ecse211.finalproject.Resources.redTeam;

import java.util.ArrayList;
import java.util.Arrays;

import ca.mcgill.ecse211.finalproject.Navigation.TravelingMode;
import ca.mcgill.ecse211.finalproject.phase2.ColorPoller;
import ca.mcgill.ecse211.finalproject.phase2.PathFinder;
//import ca.mcgill.ecse211.finalproject.phase1.UltrasonicLocalizer;
import lejos.hardware.Button;
import lejos.hardware.Sound;

/**
 * The main driver class for the odometry lab.
 */
public class Main {
	public static boolean localizationFinished = false;
	public static ArrayList<int[]> moves;

	/**
	 * The main entry point.
	 * 
	 * @param args
	 */
	public static void main(String[] args) {
		System.out.println("Hello world!");
		//--===[PHASE 1: LOCALOZATION]===--//
		Thread USPollerThread = new Thread(Resources.usPoller);
		Thread USLocalizerThread = new Thread(Resources.usLocalizer);
		odometer.start();
		USPollerThread.start();
		USLocalizerThread.start();
		while (!localizationFinished) {
			Main.sleepFor(500);
		}
		
		//beep after localization
		Sound.beep();
		sleepFor(100);
		Sound.beep();
		sleepFor(100);
		Sound.beep();
		sleepFor(100);
		
		try {
			USLocalizerThread.join(1000);
		} catch (InterruptedException e) {
			System.out.println("Sadness is the ichor of life");
		}
		
		Resources.shooterMotor.setSpeed(Resources.SHOOTER_MOTOR_SPEED);
		Resources.shooterMotor.setAcceleration(9999);
		Resources.pathFinder = new PathFinder(redTeam == TEAM_NUMBER);
		//low error threshold to prevent large thread fuck-ups
		Resources.leftMotor.setStallThreshold(10, 10);
		Resources.rightMotor.setStallThreshold(10, 10);

		//--===[PHASE 2: TRAVEL TO A LAUNCH POINT]===--//
		Thread cT = new Thread(colorPoller);
		cT.start();
		//stressShooterTest();
		// stressTest();
		// Resources.pathFinder.printMap();
		moves = PathFinder.findPath();
		boolean success = Navigation.run(moves);
		while (!success) {
			success = Navigation.run(moves);
		}
		//--===[PHASE 3: LAUNCH THE BALL]===--//
		Navigation.launchManeuver();
		//--===[PHASE 4: GO HOME]===--//
		success = Navigation.run(moves);
		while (!success) {
			success = Navigation.run(moves);
		}
		Sound.beep();
		sleepFor(100);
		Sound.beep();
		sleepFor(100);
		Sound.beep();
		sleepFor(100);
		Sound.beep();
		sleepFor(100);
		Sound.beep();
		sleepFor(100);
		Button.waitForAnyPress();
		System.exit(0);
	}

	public static void sleepFor(long duration) {
		try {
			Thread.sleep(duration);
		} catch (InterruptedException e) {
			// There is nothing to be done here
		}
	}

	public static void navigationStressTest() {
		ArrayList<int[]> moves = new ArrayList<int[]>();
		moves.add(new int[] { 0, 1 });
		moves.add(new int[] { 0, 2 });
		moves.add(new int[] { 0, 3 });
		moves.add(new int[] { 1, 3 });
		moves.add(new int[] { 1, 2 });
		moves.add(new int[] { 1, 1 });
		moves.add(new int[] { 1, 0 });
		moves.add(new int[] { 2, 0 });
		moves.add(new int[] { 2, 1 });
		moves.add(new int[] { 2, 2 });
		moves.add(new int[] { 2, 3 });
		moves.add(new int[] { 3, 3 });
		moves.add(new int[] { 3, 2 });
		moves.add(new int[] { 3, 1 });
		moves.add(new int[] { 3, 0 });
		moves.add(new int[] { 2, 0 });
		moves.add(new int[] { 1, 0 });
		moves.add(new int[] { 0, 0 });

		while (true) {
			for (int[] move : moves) {
				System.out.println(Arrays.toString(move));
				Navigation.setSpeed(Resources.LOW_FORWARD_SPEED);
				Navigation.processNextMove(move);
				while (!Navigation.moveSuccessful || Navigation.interrupted) {
					if (Navigation.navigationMode == TravelingMode.TRAVELING) {
						Navigation.processNextMove(move);
						Main.sleepFor(20);
					} else {
						Main.sleepFor(70);
					}
				}
			}
		}

	}

	public static void stressShooterTest() {
		while (true) {
			Resources.shooterMotor.rotate(165,false);
			Resources.shooterMotor.flt(true);
			Button.waitForAnyPress();
			Resources.shooterMotor.rotate(-165);
			Button.waitForAnyPress();
		}
	}
}
