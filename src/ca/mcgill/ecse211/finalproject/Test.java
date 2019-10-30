package ca.mcgill.ecse211.finalproject;

import java.util.ArrayList;

public class Test {
  public static void main(String[] args) {
    TestPathFinding();
  }


  public static void TestPathFinding() {
	  
	  Resources.leftMotor.setAcceleration(1000);
	  Resources.rightMotor.setAcceleration(1000);
	  Resources.leftMotor.setSpeed(150);
	  Resources.rightMotor.setSpeed(150);
	  
    PathFinder pf = PathFinder.test(15, 9, 4, 7, 6, 8, 0, 5, 4, 9, 6, 5, 15, 9, 12, 6);
    
    pf.setObstacle(7, 6);
	pf.setObstacle(7, 7);
	pf.setObstacle(7, 8);
	pf.setObstacle(2, 6);
	pf.setObstacle(2, 7);
	pf.setObstacle(2, 8);
    
    ArrayList<int[]> path = pf.findPath();
    for (int[] item : path) {
      Navigation.processNextMove(item);
    }
  }
}
