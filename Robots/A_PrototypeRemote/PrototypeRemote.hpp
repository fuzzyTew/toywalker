#include <TransceiverArduinoStream.hpp>

struct joystick_t
{
	double x;
	double y;
};

class PrototypeRemote : public TransceiverArduinoStream<unsigned char, joystick_t, joystick_t, joystick_t, joystick_t, bool, bool, bool, bool>
{
public:
	PrototypeRemote(Stream & stream)
	: TransceiverArduinoStream<unsigned char, joystick_t, joystick_t, joystick_t, joystick_t, bool, bool, bool, bool>(stream, _mode, _joysticks[0], _joysticks[1], _joysticks[2], _joysticks[3], _keys[0], _keys[1], _keys[2], _keys[3]),
	  _mode(0),
	  _modeMillis(millis()),
	  _joysticks{{0.5,0.5},{0.5,0.5},{0.5,0.5},{0.5,0.5}},
	  _keys{false,false,false,false}
	{ }

	static constexpr unsigned long BAUD = 38400;//19200;

	unsigned char mode() const
	{
		return _mode;
	}

	unsigned long modeMillis() const
	{
		return _modeMillis;
	}

	void incrementMode()
	{ 
		_mode = (_mode + 1) % 6;
		send(0);
		_modeMillis = millis();
	}

	joystick_t const & joystick(unsigned char index) const
	{
		return _joysticks[index];
	}

	void joystick(unsigned char index, double x, double y)
	{
		if (x != _joysticks[index].x || y != _joysticks[index].y) {
			_joysticks[index] = {x, y};
			send(1 + index);
		}
	}

	bool key(unsigned char index) const
	{
		return _keys[index];
	}

	void key(unsigned char index, bool value)
	{
		if (value != _keys[index]) {
			_keys[index] = value;
			send(5 + index);
		}
	}

	//unsigned long keyMillis(unsigned char index)
	//{
	//	return _keyMillis[index];
	//}
	
private:
	void received(unsigned char index)
	{
		if (index == 0)
			_modeMillis = millis();
	}
	unsigned char _mode;
	unsigned long _modeMillis;
	joystick_t _joysticks[4];
	bool _keys[4];
	unsigned long _keyMillis[4];
};
