package ca.mcgill.ecse211.finalproject;

import java.util.ArrayList;

public class Test {
  public static void main(String[] args) {
    // detect obstacles
    Thread obstacleDetector = new Thread(new UltrasonicObstacleDetector());
    obstacleDetector.start();
    TestPathFinding(Resources.pf);
  }


  public static void TestPathFinding(PathFinder pf) {
    // navigate
    Resources.leftMotor.setAcceleration(1000);
    Resources.rightMotor.setAcceleration(1000);
    Resources.leftMotor.setSpeed(150);
    Resources.rightMotor.setSpeed(150);
    Navigation.startNavigating();

    ArrayList<int[]> path = pf.findPath();
    for (int[] item : path) {
      if(!Navigation.getNavigating())
        break;
      Navigation.processNextMove(item);
    }
  }
}
