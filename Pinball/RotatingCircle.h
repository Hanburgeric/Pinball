#ifndef ROTATINGCIRCLE_H
#define ROTATINGCIRCLE_H

class b2World;
class b2Body;
class b2Joint;

class RotatingCircle
{
public:
	RotatingCircle(b2World* worldPtr, const float& x, const float& y, const float& r, b2Body* center);
	~RotatingCircle();

	void render();
	void update();

private:
	b2Body* bodyPtr;
	b2Joint* jointPtr;
	b2Body* centerPtr;
	const float radius;
};

#endif
