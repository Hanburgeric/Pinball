#ifndef BORDER_H
#define BORDER_H

#include <vector>

class b2World;
class b2Body;
struct b2Vec2;

enum BorderType
{
	Chain,
	Loop
};

class Border
{
public:
	Border(const BorderType& type);
	~Border();

	void addVertex(const float& x, const float& y);

	//Rotates the starting around a center by a specified number of degrees in n increments and pushes the resulting vectors.
	void addCurve(const b2Vec2& start, const b2Vec2& center, const float& deg, const int& n);

	void createBorder(b2World* worldPtr);

	b2Body* getBody();
	std::vector<b2Vec2> getVec();

	void render();

private:
	b2Body* bodyPtr;
	BorderType borderType;
	std::vector<b2Vec2> vertices;
};

#endif
