package ca.mcgill.ecse211.finalproject;

import java.util.ArrayList;

public class Test {
  public static void main(String[] args) {
    TestPathFinding();
  }


  public static void TestPathFinding() {
    PathFinder pf = PathFinder.test(15, 9, 4, 7, 6, 8, 0, 5, 4, 9, 6, 5, 15, 9, 12, 6);
    pf.printMap();
    
    ArrayList<int[]> path = pf.findPath();
    for (int[] item : path) {
      Navigation.processNextMove(item);
    }
  }
}
