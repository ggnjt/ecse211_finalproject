package ca.mcgill.ecse211.finalproject.phase1;

import static ca.mcgill.ecse211.finalproject.Resources.*;

import ca.mcgill.ecse211.finalproject.Main;

/**
 * Thread which is responsible for using the US sensor and the wall rammer to
 * localize in the beginning of the competition
 * 
 * @author yp
 *
 */
public class UltrasonicLocalizer implements Runnable {

	private static boolean kill = false;

	/**
	 * states when going through localization
	 * 
	 * @author yp
	 *
	 */
	enum SearchingState {
		/**
		 * first state when the US sensor is still buffering
		 */
		INIT,
		/**
		 * When the robot face away from the walls
		 */
		GAZING_THE_ABYSS,
		/**
		 * when the robot faces the y-axis wall
		 */
		YWALL,
		/**
		 * The robot rams the Y-wall
		 */
		RAM_Y, BACK_Y,
		/**
		 * The robot rams the X-wall
		 */
		RAM_X,
		/**
		 * The robot backs off from the X-wall
		 */
		FINISHING,
		/**
		 * The robot stops
		 */
		FINISHED;
	};

	/**
	 * current state of the robot
	 */
	public static SearchingState state = SearchingState.INIT;

	/**
	 * counter buffer used for counting derivative jumps
	 */
	private static int spaceCounter = 0;
	/**
	 * place holder to store the reading from the sensor
	 */
	private static int reading;

	@Override
	public void run() {
		navigation.setSpeed(ROTATE_SPEED);

		// start by rotating the robot counter clockwise
		rotateCounterClockWiseNonBLocking();
		while (!kill) {
			reading = usPoller.getDistance();
			if (reading == -1)
				continue;
			/**
			 * The state machine for ramming the walls to localize
			 */
			switch (state) {
			case INIT:
				init();
				break;
			case GAZING_THE_ABYSS:
				gazeTheAbyss();
				break;
			case YWALL:
				detectYWall();
				break;
			case RAM_Y:
				ramYWall();
				break;
			case BACK_Y:
				backOffFromYWall();
				break;
			case RAM_X:
				ramXWall();
				break;
			case FINISHING:
				finishing();
				break;
			case FINISHED:
				// odometer.setXYT(TILE_SIZE / 2d, TILE_SIZE / 2d, 0); //miving this to
				// pathfinder constructor
				navigation.stopTheRobot();
				kill = true;
				Main.localizationFinished = true;
				break;
			}
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

	// ===================(state methods)===================//

	/**
	 * method for the initial state, the robot turns until the reading of higher
	 * than 5 tiles away is detected, at whcih point we know that the robot is
	 * pointing away from the wall
	 */
	private static void init() {
		if (reading > TILE_SIZE * 5.0) {
			spaceCounter++;
		} else
			spaceCounter = 0;
		if (spaceCounter > 3) {
			state = SearchingState.GAZING_THE_ABYSS;
			spaceCounter = 0;
		}
	}

	/**
	 * method for when the robot is pointing away from the wall, it continues to
	 * turn until it encounters a wall, at which point it slows down
	 */
	private static void gazeTheAbyss() {
		if (reading < TILE_SIZE*0.6) {
			spaceCounter++;
		} else
			spaceCounter = 0;
		if (spaceCounter > 3) {
			state = SearchingState.YWALL;
			navigation.setSpeed(CORRECTION_SPPED); // slows the robot down to get better readings
			spaceCounter = 0;
		}
	}

	/**
	 * the method for that the robot takes to detect the y-axis wall. the robot
	 * turns very slowly until there is an increase to the reading, at which point
	 * we know that the robot is relatively perpendicular to the y-axis wall
	 */
	private static void detectYWall() {
		if (spaceCounter < 10) {
			spaceCounter++;
		} else {
			navigation.setSpeed(HIGH_FORWARD_SPEED);
			navigation.stopTheRobot();
			state = SearchingState.RAM_Y;
			spaceCounter = 0;
		}
	}

	/**
	 * The robot is pointing towards to y-axis wall and drives forward for about 5
	 * seconds, the two protruding rammers will heklp the robot to align itself
	 */
	private static void ramYWall() {
		leftMotor.forward();
		rightMotor.forward();
		spaceCounter++;
		if (spaceCounter > 35) {
			navigation.stopTheRobot();
			state = SearchingState.BACK_Y;
			spaceCounter = 0;
		}
	}

	/**
	 * the robot backs away from the wall for 5 cm and turns counter-clock-wise to
	 * face the x-axis wall.
	 */
	private static void backOffFromYWall() {
		navigation.setSpeed(HIGH_FORWARD_SPEED);
		leftMotor.rotate(navigation.convertDistance(-5.0), true);
		rightMotor.rotate(navigation.convertDistance(-5.0), false);
		leftMotor.rotate(navigation.convertAngle(-90.0), true);
		rightMotor.rotate(navigation.convertAngle(90.0), false);
		navigation.stopTheRobot();
		state = SearchingState.RAM_X;
	}

	/**
	 * the robot drive forward for about 7.5 seconds to ram the x-axis wall. again
	 * the protruding rammers will help the robot to align with x-wall
	 */
	private static void ramXWall() {
		leftMotor.forward();
		rightMotor.forward();
		spaceCounter++;
		if (spaceCounter > 35) {
			navigation.stopTheRobot();
			state = SearchingState.FINISHING;
			spaceCounter = 0;
		}
	}

	/**
	 * the robot back up from the x-axis wall and turns clock-wise 180 degrees.
	 * After this statge the robot will be at the center of the square and facing
	 * exactly 0 degrees
	 */
	private static void finishing() {
		leftMotor.rotate(navigation.convertDistance(-5.0), true);
		rightMotor.rotate(navigation.convertDistance(-5.0), false);
		leftMotor.rotate(navigation.convertAngle(180.0), true);
		rightMotor.rotate(navigation.convertAngle(-180.0), false);
		// rotate clockwise to avoid running into the wall here
		state = SearchingState.FINISHED;
	}

	// ==================(motion methods)==================//

	/**
	 * rotate counter clock wise
	 */
	private static void rotateCounterClockWiseNonBLocking() {
		leftMotor.backward();
		rightMotor.forward();
	}
	
	public static void kill() {
		kill = true;
	}
}
