#ifndef ROTATINGPLATFORM_H
#define ROTATINGPLATFORM_H

#include <vector>

class b2World;
class b2Body;
struct b2Vec2;

class RotatingPlatform
{
public:
	RotatingPlatform(b2World* worldPtr, const float& x, const float& y, const float& halfW, const float& halfL);
	~RotatingPlatform();

	void render();

private:
	b2Body* bodyPtr;
	std::vector<b2Vec2> vertices;
};

#endif
