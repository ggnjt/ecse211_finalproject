package ca.mcgill.ecse211.finalproject;

import static ca.mcgill.ecse211.finalproject.Resources.*;

import java.util.ArrayList;

import ca.mcgill.ecse211.finalproject.Navigation.TravelingMode;
import ca.mcgill.ecse211.finalproject.phase1.UltrasonicLocalizer;
//import ca.mcgill.ecse211.finalproject.phase1.UltrasonicLocalizer;
import lejos.hardware.Button;
import lejos.hardware.Sound;

/**
 * The main driver class for the odometry lab.
 */
public class Main {
	public static final int TARGETX = 6;
	public static final int TARGETY = 3;
	public static boolean P1finished = false;

	/**
	 * The main entry point.
	 * 
	 * @param args
	 */
	public static void main(String[] args) {
		Thread b = new Thread(usLocalizer);
		//Thread a = new Thread(USdisplay);
		Thread c = new Thread(usPoller);
		b.start();
		c.start();
		
		while (!P1finished) {
			Main.sleepFor(2000);
		}
		
		UltrasonicPoller.kill = true;
		
		try {
				b.join(5000);
				c.join(5000);
				System.out.println("success!");
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
//		Sound.beepSequenceUp();	
//		LCD.clear();
	//	Thread lcd = new Thread (Resources.display);
	//	lcd.start();
		Thread cT = new Thread(colorPoller);
		cT.start();

		ArrayList<int[]> moves = new ArrayList<int[]>();
		moves.add(new int[] { 0, 1 });
		moves.add(new int[] { 0, 2 });
		moves.add(new int[] { 0, 3 });

		moves.add(new int[] { 1, 3 });
		moves.add(new int[] { 2, 3 });
		moves.add(new int[] { 3, 3 });

		moves.add(new int[] { 3, 2 });
		moves.add(new int[] { 3, 1 });

		moves.add(new int[] { 3, 0 });
		moves.add(new int[] { 2, 0 });
		moves.add(new int[] { 1, 0 });
		moves.add(new int[] { 0, 0 });
		for (int[] move : moves) {
			navigation.processNextMove(move);
			
			while (!navigation.moveSuccessful) {
				if (navigation.navigationMode == TravelingMode.TRAVELING) {
					navigation.processNextMove(move);
					Sound.beepSequenceUp();
				}
				else {
					Main.sleepFor(200);
				}
			}
		}
		
		
		// leftMotor.setSpeed(100);
		// rightMotor.setSpeed(100);
		//
		// leftMotor.backward();
		// rightMotor.backward();
		// Sound.beep();
		//
		//
		// PathFinder pf = PathFinder.test(15, 9, 4, 7, 6, 8, 0, 5, 4, 9, 6, 5, 15, 9,
		// 12, 6);
		// pf.setObstacle(7, 6);
		// pf.setObstacle(7, 7);
		// pf.setObstacle(7, 8);
		// pf.printMap();
		//
		// ArrayList<int[]> lel = pf.findPath();
		//
		// for (int[] lol : lel) {
		//
		// }
		//
		// pf.setObstacle(9, 5);
		// pf.setObstacle(9, 6);
		// pf.setObstacle(9, 7);
		// PathFinder.resetMap();
		// pf.printMap();
		//
		// lel = pf.findPath();
		// for (int[] lol : lel) {
		//
		// }

		Button.waitForAnyPress();

		// ==== Phase 2: Navigate to position and take aim ==== //
		// killing all previous threads
		// UltrasonicPoller.kill = true;
		// UltrasonicLocalizerDisplay.kill = true;
		// try {
		// pollerThread.join(5000);
		// localizerThread.join(5000);
		// localizerDisplayThread.join(5000);
		// } catch (InterruptedException e1) {
		// e1.printStackTrace();
		// }

		// Navigate
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
}
