#include <Arduino.h>

// ########################## DEFINES ##########################
#define START_FRAME 0xABCD  // [-] Start frme definition for reliable serial communication

#ifndef HoverSerial
    #define HoverSerial Serial1 // specify serial interface
#endif

typedef struct {
    uint16_t start;
    int16_t steer;
    int16_t speed;
    uint8_t buzzerFreq;
    uint16_t checksum;
} SerialCommand;

typedef struct {
    uint16_t start;
    int16_t cmd1;
    int16_t cmd2;
    int16_t speedR_meas;
    int16_t speedL_meas;
    int16_t batVoltage;
    int16_t boardTemp;
    uint16_t cmdLed;
    uint16_t checksum;
} SerialFeedback;


class Hoverboard {
    uint8_t idx = 0;         // Index for new data pointer
    uint16_t bufStartFrame;  // Buffer Start Frame
    byte *p;                 // Pointer declaration for the new received data
    byte incomingByte;
    byte incomingBytePrev;
    SerialFeedback NewFeedback; // temporary buffer for new data

    public:
    int buzzerFreqCmd = 0;
    int speedCmd = 0;
    int steerCmd = 0;
    SerialFeedback feedback;
    
    void sendCommand();
    void receiveTelemetry();
    Hoverboard();
};

Hoverboard::Hoverboard() {
    HoverSerial.begin(115200);
}

// ########################## SEND ##########################
void Hoverboard::sendCommand() {
    SerialCommand Command;

    // Create command
    Command.start = (uint16_t) START_FRAME;
    Command.speed = (int16_t) speedCmd;
    Command.steer = (int16_t) steerCmd;
    Command.buzzerFreq = (uint8_t) buzzerFreqCmd;
    Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed ^ Command.buzzerFreq);

    // Write to Serial
    HoverSerial.write((uint8_t *)&Command, sizeof(Command));
}

// ########################## RECEIVE ##########################
void Hoverboard::receiveTelemetry() {

    // Check for new data availability in the Serial buffer
    if (HoverSerial.available()) {
        incomingByte = HoverSerial.read();                                   // Read the incoming byte
        bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev;  // Construct the start frame
    } else {
        return;
    }

    // Copy received data
    if (bufStartFrame == START_FRAME) {  // Initialize if new data is detected
        p = (byte *)&NewFeedback;
        *p++ = incomingBytePrev;
        *p++ = incomingByte;
        idx = 2;
    } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
        *p++ = incomingByte;
        idx++;
    }

    // Check if we reached the end of the package
    if (idx == sizeof(SerialFeedback)) {
        uint16_t checksum;

        checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

        // Check validity of the new data
        if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
            // Copy the new data
            memcpy(&feedback, &NewFeedback, sizeof(SerialFeedback));
        } else {
            Serial.println("Non-valid data skipped");
        }
        idx = 0;  // Reset the index (it prevents to enter in this if condition in the next cycle)
    }

    // Update previous states
    incomingBytePrev = incomingByte;
}

// ########################## LOOP ##########################

// void loop(void)
// {
//   unsigned long timeNow = millis();

//   // Check for new received data
//   Receive();

//   // Send commands
//   if (iTimeSend > timeNow) return;
//   iTimeSend = timeNow + TIME_SEND;
//   Send(0, iTest);

//   // Calculate test command signal
//   iTest += iStep;

//   // invert step if reaching limit
//   if (iTest >= SPEED_MAX_TEST || iTest <= -SPEED_MAX_TEST)
//     iStep = -iStep;

//   // Blink the LED
//   digitalWrite(LED_BUILTIN, (timeNow%2000)<1000);
// }

// ########################## END ##########################