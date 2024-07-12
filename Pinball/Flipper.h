#ifndef FLIPPER_H
#define FLIPPER_H

#include <vector>

class b2World;
class b2Body;
class b2Joint;
struct b2Vec2;

class Border;

enum FlipperType
{
	Left,
	Right
};

class Flipper
{
public:
	Flipper(b2World* worldPtr, const FlipperType& type, const float& x, const float& y, const float& halfW, const float& halfL);
	~Flipper();

	//Creates a joint at the nth index of a b2Body vertices array to which the launcher will be bound by a distance joint
	void createJoint(b2World* worldPtr, Border* borderPtr, const int& n);

	b2Body* getBody();

	void render();

private:
	b2Body* bodyPtr;
	b2Body* jointBodyPtr;
	b2Joint* jointPtr;
	FlipperType flipperType;
	std::vector<b2Vec2> vertices;
};

#endif
