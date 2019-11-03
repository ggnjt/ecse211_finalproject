package ca.mcgill.ecse211.finalproject;

public class UltrasonicObstacleDetector implements Runnable {
  private static int spaceCounter = 0; // buffer for counting derivative jumps
  private static int reading; // place holder to store the reading from the sensor
  public static boolean kill = false;
  public static boolean obstacleDetected = false;

  @Override
  public void run() {
    while (!kill) {
      reading = Resources.usPoller.getDistance();
      if (reading == -1)
        continue;

      if (reading < Resources.TILE_SIZE && spaceCounter < 3) {
        spaceCounter++;
      } else if (spaceCounter == 3) {
        obstacleDetected = true;
        spaceCounter = 0;
      }
      try {
        Thread.sleep(50);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
  }
}
