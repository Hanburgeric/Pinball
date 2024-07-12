#ifndef PORTAL_H
#define PORTAL_H

class b2World;
class b2Body;

enum PortalType
{
	Entry,
	Exit
};

class Portal
{
public:
	Portal(b2World* worldPtr, const PortalType& type, const float& x, const float& y, const float& r);
	~Portal();

	b2Body* getBody();
	PortalType getPortalType();
	float getRadius();

	void render();

private:
	b2Body* bodyPtr;
	PortalType portalType;
	const float radius;
};

#endif
