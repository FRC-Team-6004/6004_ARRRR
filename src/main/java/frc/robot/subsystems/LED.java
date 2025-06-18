package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LED {
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private boolean isRed = true; // State to toggle between red and blue

    public LED() {
        // Initialize the LED on PWM port 9
        m_led = new AddressableLED(9);

        // Reuse buffer
        // Default to a length of 150, start empty output
        m_ledBuffer = new AddressableLEDBuffer(60);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
        setColor(255, 0, 0); // Set to red
    }
        // Method to set the LED color
        public void setColor(int r, int g, int b) {
            for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, r, g, b);
            }
            m_led.setData(m_ledBuffer);
        }
    
        // Periodic method to toggle between red and blue
        public void periodic() {
            if (isRed) {
                setColor(255, 0, 0); // Set to red
            } else {
                setColor(0, 0, 255); // Set to blue
            }
            isRed = !isRed; // Toggle the color state
            
        }
 
}