package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LED extends SubsystemBase {
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
            m_ledBuffer = new AddressableLEDBuffer(300 - 38);
            m_led.setLength(m_ledBuffer.getLength());
    
            // Set the data
            m_led.setData(m_ledBuffer);
            m_led.start();
            //setColor(255, 0, 0); // Set to red
            for (int i = 0; i < m_ledBuffer.getLength(); i++) {
              m_ledBuffer.setRGB(i, 255, 255, 255);
              }
          m_led.setData(m_ledBuffer);
    }
    
    public void setColor(int r, int g, int b) {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
        m_led.setData(m_ledBuffer);
    }
     double c = 0;
      public void periodic() {
        if (edu.wpi.first.wpilibj.DriverStation.getMatchTime() < 15 && 
            edu.wpi.first.wpilibj.DriverStation.getMatchTime() > -2) {
          fox();
        } else {
          if (grabSubsystem.CoralDetect) {
            if ((c < 5) || (c < 15 && c > 10)) {
              setColor(0, 0, 0);
            } else {
              setColor(0, 255, 0);
            }
            c++;
          } else {
            setColor(255, 0, 0);
            c = 0;
          }
        }
      }
    
      public int ranI() {
        return (int) (Math.random() * 255);
      }
    
      public void rainbow() {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
          int hue = (int) (c * 9 + (i * 360 / m_ledBuffer.getLength())) % 360; // Faster and smoother rainbow effect
          double wave = Math.sin((c + i) * 0.2) * 0.5 + 0.5; // Wave-like pulsating brightness
          double sparkle = Math.random() < 0.02 ? 1.0 : wave; // Add occasional sparkles
          int r = (int) (Math.sin(0.024 * hue + 0) * 100 * sparkle + 100); // Reduced brightness
          int g = (int) (Math.sin(0.024 * hue + 2) * 100 * sparkle + 100); // Reduced brightness
          int b = (int) (Math.sin(0.024 * hue + 4) * 100 * sparkle + 100); // Reduced brightness
          m_ledBuffer.setRGB(i, r, g, b);
        }
        m_led.setData(m_ledBuffer);
        c += 1;
      }
    
      public void fox() {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
          if ((int) (((i + c) / 5) % 3) == 0) {
            m_ledBuffer.setRGB(i, 255, 40, 0); // Orange for even indices
          } else if ((int) (((i + c) / 5) % 3) == 1) {
            m_ledBuffer.setRGB(i, 255, 255, 255); // White for odd indices
          } else {
            m_ledBuffer.setRGB(i, 2, 10, 10); // White for odd indices
    
          }
        }
        m_led.setData(m_ledBuffer);
        c += 0.5;
      }
    
      public void test() {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
          if (i <= (m_ledBuffer.getLength() * tesController.getRightTriggerAxis())) {
            m_ledBuffer.setRGB(i, 255, 255, 255); // White for odd indices
          } else {
            m_ledBuffer.setRGB(i, 2, 10, 10); // White for odd indices
          }
        }
        System.out.println(m_ledBuffer.getLength() * tesController.getRightTriggerAxis());
      }
 
}