package ca.mcgill.ecse211.finalproject;

import static ca.mcgill.ecse211.finalproject.Resources.TEAM_NUMBER;
import static ca.mcgill.ecse211.finalproject.Resources.colorPoller;
import static ca.mcgill.ecse211.finalproject.Resources.odometer;
import static ca.mcgill.ecse211.finalproject.Resources.redTeam;

import java.util.ArrayList;
import java.util.Arrays;

import ca.mcgill.ecse211.finalproject.Navigation.TravelingMode;
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

		Thread USPollerThread = new Thread(Resources.usPoller);
		Thread USLocalizerThread = new Thread(Resources.usLocalizer);
		odometer.start();
		USPollerThread.start();
		USLocalizerThread.start();
		while (!localizationFinished) {
			Main.sleepFor(1000);
		}
		
		Sound.beepSequence();

		UltrasonicPoller.sleep();// this should be removed after demo

		try {
			USLocalizerThread.join(5000);
		} catch (InterruptedException e) {
			System.out.println("Sadness is the ichor of life");
		}
		
		Resources.shooterMotor.setSpeed(Resources.SHOOTER_MOTOR_SPEED);
		Resources.pathFinder = new PathFinder(redTeam == TEAM_NUMBER);

		Resources.leftMotor.setStallThreshold(10, 10);
		Resources.rightMotor.setStallThreshold(10, 10);

		Thread cT = new Thread(colorPoller);
		cT.start();
		// stressShooterTest();
		// stressTest();
		moves = Resources.pathFinder.findPath();
		// Resources.pathFinder.printMap();

		boolean success = Navigation.run(moves);
		while (!success) {
			success = Navigation.run(moves);
		}

		Navigation.goToLowerLeftCorner();
		Navigation.turnTo(PathFinder.launchAngle + 180);
		Navigation.moveByDistance((-PathFinder.launchAdjustment)*Resources.TILE_SIZE - 0.5 * Resources.TILE_SIZE);
		System.out.println("tile: " + Navigation.xTile +"==" + Navigation.yTile);
		System.out.println("Angle: " + PathFinder.launchAngle);
		Resources.shooterMotor.rotate(165);
		Resources.shooterMotor.flt(true);
		Resources.pathFinder.printMap();
		PathFinder.letsGoHome();
		Navigation.moveByDistance((PathFinder.launchAdjustment)*Resources.TILE_SIZE + 0.5 * Resources.TILE_SIZE);
		Navigation.reCenter();
		Main.sleepFor(100);
		moves = Resources.pathFinder.findPath();
		success = Navigation.run(moves);
		while (!success) {
			success = Navigation.run(moves);
		}
//		colorPoller.sleep();
//		navigation.goToLowerLeftCorner();
//		navigation.turnTo(Resources.targetAngle);
//		Sound.beep();
//		Main.sleepFor(500);
//		Sound.beep();
//		Main.sleepFor(500);
//		Sound.beep();
//		Resources.shooterMotor.rotate(165);
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
			Resources.shooterMotor.rotate(165);
			Resources.shooterMotor.flt(true);
			Resources.shooterMotor.rotate(-165);
			Button.waitForAnyPress();
		}
	}
}
