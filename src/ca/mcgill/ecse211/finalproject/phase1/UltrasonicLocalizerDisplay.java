//package ca.mcgill.ecse211.finalproject.phase1;
//
//import static ca.mcgill.ecse211.finalproject.Resources.LCD;
//import static ca.mcgill.ecse211.finalproject.Resources.usPoller;
//
///**
// * display used during localization to display the states and US reading
// * 
// * @author alhomsi
// *
// */
//public class UltrasonicLocalizerDisplay implements Runnable {
//
//  /**
//   * display refresh rate in ms
//   */
//  private final long DISPLAY_PERIOD = 500;
//  /**
//   * global kill switch for the thread
//   */
//  public static boolean kill = false;
//  /**
//   * max timeout value for the thread
//   */
//  private long timeout = Long.MAX_VALUE;
//
//  public void run() {
//    LCD.clear();
//    long updateStart, updateEnd;
//    long tStart = System.currentTimeMillis();
//    do {
//      LCD.clear();
//      if (kill)
//        break;
//      updateStart = System.currentTimeMillis();
//      LCD.drawString(UltrasonicLocalizer.state.toString(), 0, 0);
//      LCD.drawString("" + usPoller.getDistance(), 0, 1);
//      updateEnd = System.currentTimeMillis();
//      if (updateEnd - updateStart < DISPLAY_PERIOD) {
//        try {
//          Thread.sleep(DISPLAY_PERIOD - (updateEnd - updateStart));
//        } catch (InterruptedException e) {
//          e.printStackTrace();
//        }
//      }
//    } while ((updateEnd - tStart) <= timeout);
//
//  }
//
//  /**
//   * Sets the timeout in ms.
//   * 
//   * @param timeout
//   */
//  public void setTimeout(long timeout) {
//    this.timeout = timeout;
//  }
//
//  /**
//   * Shows the text on the LCD, line by line.
//   * 
//   * @param strings comma-separated list of strings, one per line
//   */
//  public static void showText(String... strings) {
//    LCD.clear();
//    for (int i = 0; i < strings.length; i++) {
//      LCD.drawString(strings[i], 0, i);
//    }
//  }
//
//}
