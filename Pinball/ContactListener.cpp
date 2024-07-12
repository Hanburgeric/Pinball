#include "ContactListener.h"

void ContactListener::BeginContact(b2Contact* contactPtr)
{
	b2Fixture* fixtureA = contactPtr->GetFixtureA();
	b2Fixture* fixtureB = contactPtr->GetFixtureB();

	if (fixtureA->IsSensor() && fixtureB->GetBody()->GetType() == b2_dynamicBody)
	{
		fixturePtrPairs.insert(std::make_pair(fixtureB, fixtureA));
	}

	else if (fixtureA->GetBody()->GetType() == b2_dynamicBody && fixtureB->IsSensor())
	{
		fixturePtrPairs.insert(std::make_pair(fixtureA, fixtureB));
	}
}

void ContactListener::EndContact(b2Contact* contactPtr)
{
	b2Fixture* fixtureA = contactPtr->GetFixtureA();
	b2Fixture* fixtureB = contactPtr->GetFixtureB();

	if (fixtureA->IsSensor() && fixtureB->GetBody()->GetType() == b2_dynamicBody)
	{
		fixturePtrPairs.erase(std::make_pair(fixtureB, fixtureA));
	}

	else if (fixtureA->GetBody()->GetType() == b2_dynamicBody && fixtureB->IsSensor())
	{
		fixturePtrPairs.erase(std::make_pair(fixtureA, fixtureB));
	}
}
