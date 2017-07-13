#include "PrototypeRemote.hpp"

void setup()
{
	Serial.begin(PrototypeRemote::BAUD);
	PrototypeRemote remote(Serial);
	
	pinMode(2, INPUT_PULLUP);

	for (int i = 3; i < 20; ++ i)
		pinMode(i, INPUT);
	
	bool rotary = digitalRead(2);
	for (;;) {
		bool nextRotary = digitalRead(2);
		if (nextRotary != rotary && millis() - remote.modeMillis() > 100) {
			remote.incrementMode();
			//Serial.print("Click\n");
			//Serial.println(remote.mode());
			rotary = nextRotary;
		}

		remote.joystick(0, analogRead(0) / 1024.0, analogRead(1) / 1024.0);
		/*for (int i = 0; i < 4; ++ i) {
			remote.joystick(i, analogRead(i*2) / 1024.0, analogRead(i*2+1) / 1024.0);
			remote.key(i, !digitalRead(3+i));
		}*/
	}
}

void loop()
{
}
