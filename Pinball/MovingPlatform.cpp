#include "Box2D/Box2D.h"
#include "GL/freeglut.h"

#include "MovingPlatform.h"

MovingPlatform::MovingPlatform(b2World* worldPtr, const float& x, const float& y, const float& halfW, const float& halfL) :
	bodyPtr()
{
	b2BodyDef bodyDef;
	bodyDef.type = b2_kinematicBody;
	bodyDef.position.Set(x, y);
	bodyPtr = worldPtr->CreateBody(&bodyDef);

	b2PolygonShape shape;
	shape.SetAsBox(halfW, halfL);
	for (int index = 0; index < shape.m_count; index++)
	{
		vertices.push_back(b2Vec2(shape.m_vertices[index]));
	}

	b2FixtureDef fixture;
	fixture.shape = &shape;
	fixture.density = 0.75f;
	fixture.friction = 0.75f;
	fixture.restitution = 0.75f;
	bodyPtr->CreateFixture(&fixture);
}

MovingPlatform::~MovingPlatform()
{
	bodyPtr = nullptr;
}

void MovingPlatform::render()
{
	glPushMatrix();
	glTranslatef(bodyPtr->GetPosition().x, bodyPtr->GetPosition().y, 0.0f);
	glRotatef(bodyPtr->GetAngle() * 180.0f / b2_pi, 0.0f, 0.0f, 1.0f);
	glColor3f(0.863f, 0.847f, 0.753f);

	glBegin(GL_QUADS);
	for (int index = 0; index < vertices.size(); index++)
	{
		glVertex2f(vertices[index].x, vertices[index].y);
	}
	glEnd();
	glPopMatrix();
}

void MovingPlatform::update()
{
	if (bodyPtr->GetPosition().x <= 14.0f)
	{
		bodyPtr->SetLinearVelocity(b2Vec2(5.0f, 0.0f));
	}

	else if (bodyPtr->GetPosition().x >= 20.0f)
	{
		bodyPtr->SetLinearVelocity(b2Vec2(-5.0f, 0.0f));
	}
}
