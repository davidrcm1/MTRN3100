#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32


class Display {
public:
    Display() : screen(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET) {}

    void setup() {
        //Wire.begin();
        //Serial.begin(9600);

        // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
        if (!screen.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
            Serial.println(F("SSD1306 allocation failed"));
            for (;;) ; // Don't proceed, loop forever
        }

        // Show initial display buffer contents on the screen --
        // the library initializes this with an Adafruit splash screen.
        screen.display();
        delay(2000); // Pause for 2 seconds

        // Clear the buffer
        screen.clearDisplay();

        // Draw a single pixel in white
        screen.drawPixel(10, 10, SSD1306_WHITE);

        // Show the display buffer on the screen. You MUST call display() after
        // drawing commands to make them visible on screen!
        screen.display();
        delay(2000);
    }

    void printString(const char* buf) {
      screen.clearDisplay();

      screen.setTextSize(1);             // Normal 1:1 pixel scale
      screen.setTextColor(SSD1306_WHITE);        // Draw white text
      screen.setCursor(0,0);             // Start at top-left corner
      screen.println(buf);

      screen.display();
      delay(2000);
        // screen.clearDisplay();
        // screen.setTextSize(1);
        // screen.setCursor(0, 0);
        // screen.println(buf);
        // screen.display();
    }

    void testdrawstyles(void) {
      screen.clearDisplay();

      screen.setTextSize(1);             // Normal 1:1 pixel scale
      screen.setTextColor(SSD1306_WHITE);        // Draw white text
      screen.setCursor(0,0);             // Start at top-left corner
      screen.println(F("Hello, world!"));

      screen.display();
      delay(2000);
    }

    void testdrawstyles2(void) {
      screen.clearDisplay();

      screen.setTextSize(1);             // Normal 1:1 pixel scale
      screen.setTextColor(SSD1306_WHITE);        // Draw white text
      screen.setCursor(0,0);             // Start at top-left corner
      screen.println(F("bro wat"));

      screen.display();
      delay(2000);
    }

private:
    Adafruit_SSD1306 screen;
};
