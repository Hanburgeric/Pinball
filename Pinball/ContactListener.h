#ifndef CONTACTLISTENER_H
#define CONTACTLISTENER_H

#include "Box2D/Box2D.h"
#include <set>
#include <utility>

class ContactListener : public b2ContactListener
{
public:
	void BeginContact(b2Contact* contactPtr);
	void EndContact(b2Contact* contactPtr);

	std::set<std::pair<b2Fixture*, b2Fixture*>> fixturePtrPairs;
};

#endif
