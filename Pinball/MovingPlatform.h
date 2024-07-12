#ifndef MOVINGPLATFORM_H
#define MOVINGPLATFORM_H

#include <vector>

class b2World;
class b2Body;
struct b2Vec2;

class MovingPlatform
{
public:
	MovingPlatform(b2World* worldPtr, const float& x, const float& y, const float& w, const float& l);
	~MovingPlatform();

	void render();
	void update();

private:
	b2Body* bodyPtr;
	std::vector<b2Vec2> vertices;
};

#endif
