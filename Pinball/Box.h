#ifndef BOX_H
#define BOX_H

#include <vector>

class b2World;
class b2Body;
struct b2Vec2;

class Box
{
public:
	Box(b2World* worldPtr, const float& x, const float& y, const float& halfW, const float& halfL);
	~Box();

	void render();

private:
	b2Body* bodyPtr;
	std::vector<b2Vec2> vertices;
};

#endif
