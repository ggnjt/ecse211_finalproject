package ca.mcgill.ecse211.finalproject.test;

import static ca.mcgill.ecse211.finalproject.Resources.*;
import ca.mcgill.ecse211.finalproject.Resources;
import lejos.hardware.Button;

/**
 * Example project using WifiConnection to communicate with a server and receive data concerning the
 * competition such as the starting corner the robot is placed in.<br>
 * 
 * <p>Keep in mind that this class is an <b>example</b> of how to use the Wi-Fi code; you must use
 * the WifiConnection class yourself in your own code as appropriate. In this example, we simply
 * show how to get and process different types of data.<br>
 * 
 * <p>There are two variables you MUST set manually (in Resources.java) before trying to use this
 * code:
 * 
 * <ol>
 * <li>SERVER_IP: The IP address of the computer running the server application. This will be your
 * own laptop, until the beta beta demo or competition where this is the TA or professor's laptop.
 * In that case, set the IP to the default (indicated in Resources).</li>
 * <li>TEAM_NUMBER: your project team number.</li>
 * </ol>
 * 
 * <p>Note: We use {@code System.out.println()} instead of LCD printing so that full debug output 
 * (eg, the very long string containing the transmission) can be read on the screen OR a remote 
 * console such as the EV3Control program via Bluetooth or Wi-Fi. You can disable printing from the 
 * Wi-Fi code via ENABLE_DEBUG_WIFI_PRINT.
 * 
 * @author Michael Smith, Tharsan Ponnampalam, Younes Boubekeur
 */
public class WifiExample {
  
  /**
   * The main method.
   * @param args not used
   */
  public static void main(String[] args) {
    Resources.receiveWifiParameters();
   
    System.out.println("Running...");

    // Example 1: Print out all received data
    System.out.println("Map:\n" + wifiParameters);

    // Example 2: Print out specific values
    System.out.println("Red Team: " + redTeam);
    System.out.println("Green Zone: " + green);
    System.out.println("Island Zone, upper right: " + island.ur);
    System.out.println("Red tunnel footprint, lower left y value: " + tnr.ll.y);

    // Example 3: Compare value;
    if (bin.x < 5) {
      System.out.println("Bin location X < 5");
    } else {
      System.out.println("Bin location X >= 5");
    }

    // Wait until user decides to end program
    Button.waitForAnyPress();
  }
}