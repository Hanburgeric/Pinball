#include "Box2D/Box2D.h"
#include "GL/freeglut.h"

#include "Water.h"

Water::Water(b2World* worldPtr, const float& x, const float& y, const float& halfW, const float& halfL) :
	bodyPtr()
{
	b2BodyDef bodyDef;
	bodyDef.type = b2_staticBody;
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
	fixture.density = 1.0f;
	fixture.friction = 1.0f;
	fixture.isSensor = true;
	bodyPtr->CreateFixture(&fixture);
}

Water::~Water()
{
	bodyPtr = nullptr;
}

void Water::render()
{
	glPushMatrix();
	glTranslatef(bodyPtr->GetPosition().x, bodyPtr->GetPosition().y, 0.0f);
	glRotatef(bodyPtr->GetAngle() * 180.0f / b2_pi, 0.0f, 0.0f, 1.0f);
	glColor3f(0.4f, 0.8f, 0.8f);

	glBegin(GL_QUADS);
	for (int index = 0; index < vertices.size(); index++)
	{
		glVertex2f(vertices[index].x, vertices[index].y);
	}
	glEnd();
	glPopMatrix();
}
