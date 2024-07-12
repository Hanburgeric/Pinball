#ifndef WATER_H
#define WATER_H

#include <vector>

class b2World;
class b2Body;
struct b2Vec2;

class Water
{
public:
	Water(b2World* worldPtr, const float& x, const float& y, const float& w, const float& l);
	~Water();

	void render();

private:
	b2Body* bodyPtr;
	std::vector<b2Vec2> vertices;
};

#endif
