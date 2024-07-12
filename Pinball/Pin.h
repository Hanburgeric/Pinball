#ifndef PIN_H
#define PIN_H

class b2World;
class b2Body;
class b2Shape;

class Pin
{
public:
	Pin(b2World* worldPtr, const float& x, const float& y, const float& r);
	~Pin();

	void render();

private:
	b2Body* bodyPtr;
	const float radius;
};

#endif
