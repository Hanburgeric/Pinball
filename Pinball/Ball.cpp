#include "Box2D/Box2D.h"
#include "GL/freeglut.h"

#include "Ball.h"

Ball::Ball(b2World* worldPtr, const float& x, const float& y, const float& r) :
	bodyPtr(),
	radius(r)
{
	b2BodyDef bodyDef;
	bodyDef.type = b2_dynamicBody;
	bodyDef.position.Set(x, y);
	bodyPtr = worldPtr->CreateBody(&bodyDef);

	b2CircleShape shape;
	shape.m_radius = radius;

	b2FixtureDef fixture;
	fixture.shape = &shape;
	fixture.density = 0.75f;
	fixture.friction = 0.75f;
	fixture.restitution = 0.25f;
	bodyPtr->CreateFixture(&fixture);
}

Ball::~Ball()
{
	bodyPtr = nullptr;
}

b2Body* Ball::getBody()
{
	return bodyPtr;
}

float Ball::getRadius()
{
	return radius;
}

void Ball::render()
{
	glPushMatrix();
	glTranslatef(bodyPtr->GetPosition().x, bodyPtr->GetPosition().y, 0.0f);
	glRotatef(bodyPtr->GetAngle() * 180.0f / b2_pi, 0.0f, 0.0f, 1.0f);
	glColor3f(0.988f, 0.416f, 0.012f);

	glBegin(GL_TRIANGLE_FAN);
	glVertex2f(0.0f, 0.0f);
	for (int index = 2; index < 98; index++)
	{
		glVertex2f(radius * cos(2.0f * b2_pi * (float)index / 100.0f), radius * sin(2.0f * b2_pi * (float)index / 100.0f));
		glVertex2f(radius * cos(2.0f * b2_pi * (float)(index + 1) / 100.0f), radius * sin(2.0f * b2_pi * (float)(index + 1) / 100.0f));
	}
	glEnd();
	glPopMatrix();
}
