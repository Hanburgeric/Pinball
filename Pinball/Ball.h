#ifndef BALL_H
#define BALL_H

class b2World;
class b2Body;

class Ball
{
public:
	Ball(b2World* worldPtr, const float& x, const float& y, const float& radius);
	~Ball();

	b2Body* getBody();
	float getRadius();

	void render();

private:
	b2Body* bodyPtr;
	const float radius;
};

#endif
