#include "Box2D/Box2D.h"
#include "GL/freeglut.h"

#include "Pin.h"

Pin::Pin(b2World* worldPtr, const float& x, const float& y, const float& r) :
	bodyPtr(),
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
	fixture.density = 0.75f;
	fixture.friction = 0.75f;
	fixture.restitution = 0.75f;
	bodyPtr->CreateFixture(&fixture);
}

Pin::~Pin()
{
	bodyPtr = nullptr;
}

void Pin::render()
{
	glPushMatrix();
	glTranslatef(bodyPtr->GetPosition().x, bodyPtr->GetPosition().y, 0.0f);
	glRotatef(bodyPtr->GetAngle() * 180.0f / b2_pi, 0.0f, 0.0f, 1.0f);
	glColor3f(0.863f, 0.847f, 0.753f);

	glutSolidSphere(radius, 100, 100);
	glPopMatrix();
}
