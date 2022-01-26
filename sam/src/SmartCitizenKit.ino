#include "SckBase.h"

#ifdef testing
#include "SckTest.h"
#endif

SckBase base;

#ifdef testing
SckTest sckTest(&base);
#endif

bool reset_pending = false;

// Led update interrupt
void TC5_Handler (void) {
	base.led.tick();
	TC5->COUNT16.INTFLAG.bit.MC0 = 1;
}
// Button events interrupt
void ISR_button() {
	base.butState = digitalRead(pinBUTTON);
#ifdef testing
	sckTest.test_button();
#else
	base.butFeedback();
#endif
}
// Card detect interrupt
void ISR_sdDetect() {
	base.sdDetect();
}
// void ISR_alarm() {
// 	base.wakeUp();
// };

void setup() {

	base.setup();

#ifdef testing
	sckTest.test_full();
#endif
}

void loop() {
	base.update();
	delay(100);
}

// the following function appears to be bound to input received on SerialUSB; 
// an event (ISR) is fixed each time there is a character typed on the UI.
// flow: serialEventRun -> base::inputUpdate -> (Command) In()
void serialEventRun() {
	base.inputUpdate();
}
