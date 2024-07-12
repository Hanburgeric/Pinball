#ifndef LAUNCHER_H
#define LAUNCHER_H

#include <vector>

class b2World;
class b2Body;
class b2Joint;
struct b2Vec2;

class Launcher
{
public:
	Launcher(b2World* worldPtr, const float& x, const float& y, const float& halfW, const float& halfL);
	~Launcher();

	//Creates a body and joint at (x, y) to which the launcher will be bound by a distance joint
	void createBodyAndJoint(b2World* worldPtr, const float& x, const float& y);

	b2Body* getBody();

	void render();

private:
	b2Body* bodyPtr;
	b2Body* jointBodyPtr;
	b2Joint* jointPtr;

	std::vector<b2Vec2> vertices;
};

#endif
