#include "Box2D/Box2D.h"
#include "GL/freeglut.h"

#include "Launcher.h"

Launcher::Launcher(b2World* worldPtr, const float& x, const float& y, const float& halfW, const float& halfL) :
	bodyPtr(),
	jointBodyPtr(),
	jointPtr()
{
	b2BodyDef bodyDef;
	bodyDef.type = b2_dynamicBody;
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
	fixture.density = 0.5f;
	fixture.friction = 0.75f;
	fixture.restitution = 0.75f;
	bodyPtr->CreateFixture(&fixture);
}

Launcher::~Launcher()
{
	bodyPtr = nullptr;
	jointBodyPtr = nullptr;
	jointPtr = nullptr;
}

void Launcher::createBodyAndJoint(b2World* worldPtr, const float& x, const float& y)
{
	b2BodyDef bodyDef;
	bodyDef.type = b2_staticBody;
	bodyDef.position.Set(x, y);
	jointBodyPtr = worldPtr->CreateBody(&bodyDef);

	b2DistanceJointDef jointDef;
	jointDef.Initialize(jointBodyPtr, bodyPtr, jointBodyPtr->GetWorldCenter(), bodyPtr->GetWorldCenter());
	jointDef.collideConnected = true;
	jointDef.frequencyHz = 0.75f;
	jointDef.dampingRatio = 0.25f;
	jointDef.length = (bodyPtr->GetPosition().y - y) / 1.5f;
	jointPtr = (b2DistanceJoint*)worldPtr->CreateJoint(&jointDef);
}

b2Body* Launcher::getBody()
{
	return bodyPtr;
}

void Launcher::render()
{
	glPushMatrix();
	glTranslatef(bodyPtr->GetPosition().x, bodyPtr->GetPosition().y, 0.0f);
	glRotatef(bodyPtr->GetAngle() * 180.0f / b2_pi, 0.0f, 0.0f, 1.0f);
	glColor3f(0.643f, 0.369f, 0.898f);

	glBegin(GL_QUADS);
	for (int index = 0; index < vertices.size(); index++)
	{
		glVertex2f(vertices[index].x, vertices[index].y);
	}
	glEnd();
	glPopMatrix();
}
