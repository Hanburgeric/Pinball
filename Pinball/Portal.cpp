#include "Box2D/Box2D.h"
#include "GL/freeglut.h"

#include "Portal.h"

Portal::Portal(b2World* worldPtr, const PortalType& type, const float& x, const float& y, const float& r) :
	bodyPtr(),
	portalType(type),
	radius(r)
{
	b2BodyDef bodyDef;
	bodyDef.type = b2_staticBody;
	bodyDef.position.Set(x, y);
	bodyPtr = worldPtr->CreateBody(&bodyDef);

	b2CircleShape shape;
	shape.m_radius = radius;

	b2FixtureDef fixture;
	fixture.shape = &shape;
	fixture.isSensor = true;
	bodyPtr->CreateFixture(&fixture);
}

Portal::~Portal()
{
	bodyPtr = nullptr;
}

b2Body* Portal::getBody()
{
	return bodyPtr;
}

PortalType Portal::getPortalType()
{
	return portalType;
}

float Portal::getRadius()
{
	return radius;
}

void Portal::render()
{
	glPushMatrix();
	glTranslatef(bodyPtr->GetPosition().x, bodyPtr->GetPosition().y, 0.0f);
	glRotatef(bodyPtr->GetAngle() * 180.0f / b2_pi, 0.0f, 0.0f, 1.0f);
	glColor3f(0.0f, 0.0f, 0.0f);

	glBegin(GL_TRIANGLE_FAN);
	glVertex2f(0.0f, 0.0f);
	for (int index = 0; index < 100; index++)
	{
		glVertex2f(radius * cos(2.0f * b2_pi * (float)index / 100.0f), radius * sin(2.0f * b2_pi * (float)index / 100.0f));
		glVertex2f(radius * cos(2.0f * b2_pi * (float)(index + 1) / 100.0f), radius * sin(2.0f * b2_pi * (float)(index + 1) / 100.0f));
	}
	glEnd();
	glPopMatrix();
}
