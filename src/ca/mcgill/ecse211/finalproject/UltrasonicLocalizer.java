package ca.mcgill.ecse211.finalproject;

import static ca.mcgill.ecse211.finalproject.Resources.FORWARD_SPEED;
import static ca.mcgill.ecse211.finalproject.Resources.LOW_SPEED;
import static ca.mcgill.ecse211.finalproject.Resources.ROTATE_SPEED;
import static ca.mcgill.ecse211.finalproject.Resources.TILE_SIZE;
import static ca.mcgill.ecse211.finalproject.Resources.TRACK;
import static ca.mcgill.ecse211.finalproject.Resources.WHEEL_RAD;
import static ca.mcgill.ecse211.finalproject.Resources.leftMotor;
import static ca.mcgill.ecse211.finalproject.Resources.odometer;
import static ca.mcgill.ecse211.finalproject.Resources.rightMotor;
import static ca.mcgill.ecse211.finalproject.Resources.usPoller;

/**
 * Thread which is responsible for using the US sensor and the wall rammer to
 * localize in the beginning of the competition
 * 
 * @author yp
 *
 */
public class UltrasonicLocalizer implements Runnable {

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
		setSpeed(ROTATE_SPEED);
		// start by rotating the robot counterclock wise
		rotateCounterClockWiseNonBLocking();
		boolean cont = true; // boolean used to break our of while loop
		while (cont) {
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
				stopTheRobot();
				cont = false;
				odometer.setXYT(TILE_SIZE / 2, 21, 0);
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
		if (reading < TILE_SIZE * 1.5) {
			spaceCounter++;
		} else
			spaceCounter = 0;
		if (spaceCounter > 3) {
			state = SearchingState.YWALL;
			setSpeed(LOW_SPEED); // slows the robot down to get better readings
			spaceCounter = 0;
		}
	}

	/**
	 * the method for that the robot takes to detect the y-axis wall. the robot
	 * turns very slowly until there is an increase to the reading, at which point
	 * we know that the robot is relatively perpendicular to the y-axis wall
	 */
	private static void detectYWall() {
		if (spaceCounter < 40) {
			spaceCounter++;
		} else {
			setSpeed(FORWARD_SPEED);
			stopTheRobot();
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
		if (spaceCounter > 110) {
			stopTheRobot();
			state = SearchingState.BACK_Y;
			spaceCounter = 0;
		}
	}

	/**
	 * the robot backs away from the wall for 5 cm and turns counter-clock-wise to
	 * face the x-axis wall.
	 */
	private static void backOffFromYWall() {
		setSpeed(FORWARD_SPEED);
		leftMotor.rotate(Navigation.convertDistance(-7.0), true);
		rightMotor.rotate(Navigation.convertDistance(-7.0), false);
		leftMotor.rotate(Navigation.convertAngle(-90.0), true);
		rightMotor.rotate(Navigation.convertAngle(90.0), false);
		stopTheRobot();
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
		if (spaceCounter > 110) {
			stopTheRobot();
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
		leftMotor.rotate(Navigation.convertDistance(-10.0), true);
		rightMotor.rotate(Navigation.convertDistance(-10.0), false);
		leftMotor.rotate(Navigation.convertAngle(180.0), true);
		rightMotor.rotate(Navigation.convertAngle(-180.0), false);
		// rotate clockwise to avoid running into the wall here
		state = SearchingState.FINISHED;
	}
	// ==================(motion methods)==================//

	/**
	 * Stop the robot
	 */
	private static void stopTheRobot() {
		leftMotor.stop(true);
		rightMotor.stop(false);
	}

	/**
	 * set Speed
	 */
	private static void setSpeed(int speed) {
		leftMotor.setSpeed(speed);
		rightMotor.setSpeed(speed);
	}

	/**
	 * rotate counter clock wise
	 */
	private static void rotateCounterClockWiseNonBLocking() {
		leftMotor.backward();
		rightMotor.forward();
	}
}
