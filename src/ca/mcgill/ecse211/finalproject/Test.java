package ca.mcgill.ecse211.finalproject;

import java.util.ArrayList;

/**
 * test and debugging class
 * @author yp
 *
 */
public class Test {
  public static void main(String[] args) {
    // detect obstacles
    Thread obstacleDetector = new Thread(new UltrasonicObstacleDetector());
    obstacleDetector.start();
    TestPathFinding(Resources.pathFinder);
  }


  public static void TestPathFinding(PathFinder pf) {
    // navigate
    Resources.leftMotor.setAcceleration(1000);
    Resources.rightMotor.setAcceleration(1000);
    Resources.leftMotor.setSpeed(150);
    Resources.rightMotor.setSpeed(150);

    ArrayList<int[]> path = pf.findPath();
    for (int[] item : path) {
      Navigation.processNextMove(item);
    }
  }
}
