package ca.mcgill.ecse211.finalproject;

import static ca.mcgill.ecse211.finalproject.Resources.LCD;
import static ca.mcgill.ecse211.finalproject.Resources.TEAM_NUMBER;
import static ca.mcgill.ecse211.finalproject.Resources.colorPoller;
import static ca.mcgill.ecse211.finalproject.Resources.navigation;
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
	public static boolean P1finished = false;

	/**
	 * The main entry point.
	 * 
	 * @param args
	 */
	public static void main(String[] args) {

//		Thread USPollerThread = new Thread(usPoller);
//		Thread USLocalizerThread = new Thread(usLocalizer);
		odometer.start();
//		USPollerThread.start();
////		USLocalizerThread.start();
//		while (!P1finished) {
//			Main.sleepFor(2000);
//		}

//		UltrasonicPoller.kill = true;
//
//		try {
//			USPollerThread.join(5000); // this should be remvoed after
//			USLocalizerThread.join(5000);
//			System.out.println("success!");
//		} catch (InterruptedException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		}

		Resources.pathFinder = new PathFinder(redTeam == TEAM_NUMBER);
		//Resources.pathFinder.printMap();		
		
		
		LCD.clear();
		Thread cT = new Thread(colorPoller);
		cT.start();

		ArrayList<int[]> moves = Resources.pathFinder.findPath();

		
		for (int[] move : moves) {
			System.out.println(Arrays.toString(move));
			navigation.processNextMove(move);

			while (!navigation.moveSuccessful) {
				if (navigation.navigationMode == TravelingMode.TRAVELING) {
					navigation.processNextMove(move);
					Sound.beepSequenceUp();
				} else {
					Main.sleepFor(100);
				}
			}
		}

		Button.waitForAnyPress();

		// ==== Phase 3: launch the ball ==== //
		// while (shots < 5) {
		// shooterMotor.rotate(-190); // cock the launcher
		// Sound.twoBeeps(); // beep for dramatic effect
		// shooterMotor.rotate(240); // shoot
		// shooterMotor.rotate(-50); // reset angle
		// Button.waitForAnyPress(); // wait for reload
		// shots++;
		// }

		System.exit(0);
	}

	public static void sleepFor(long duration) {
		try {
			Thread.sleep(duration);
		} catch (InterruptedException e) {
			// There is nothing to be done here
		}
	}

	public ArrayList<int[]> testMoves() {
		ArrayList<int[]> moves = new ArrayList<int[]>();
		moves.add(new int[] { 0, 1 });
		moves.add(new int[] { 0, 2 });
		moves.add(new int[] { 0, 3 });
		moves.add(new int[] { 1, 3 });
		moves.add(new int[] { 2, 3 });
		moves.add(new int[] { 3, 3 });
		moves.add(new int[] { 3, 4 });
		moves.add(new int[] { 3, 5 });
		moves.add(new int[] { 3, 4 });
		moves.add(new int[] { 3, 3 });
		moves.add(new int[] { 3, 2 });
		moves.add(new int[] { 3, 1 });
		moves.add(new int[] { 3, 0 });
		return moves;
	}
}
