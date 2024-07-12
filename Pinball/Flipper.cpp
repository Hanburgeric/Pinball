#include "Box2D/Box2D.h"
#include "GL/freeglut.h"

#include "Border.h"
#include "Flipper.h"

Flipper::Flipper(b2World* worldPtr, const FlipperType& type, const float& x, const float& y, const float& halfW, const float& halfL) :
	bodyPtr(),
	jointBodyPtr(),
	jointPtr(),
	flipperType(type)
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
	fixture.density = 50.0f;
	fixture.friction = 0.75f;
	fixture.restitution = 0.75f;
	bodyPtr->CreateFixture(&fixture);
}

Flipper::~Flipper()
{
	bodyPtr = nullptr;
	jointBodyPtr = nullptr;
	jointPtr = nullptr;
}

void Flipper::createJoint(b2World* worldPtr, Border* borderPtr, const int& n)
{
	b2RevoluteJointDef jointDef;
	switch (flipperType)
	{
	case Left:
		jointDef.Initialize(borderPtr->getBody(), bodyPtr, borderPtr->getVec()[n]);
		break;
	case Right:
		jointDef.Initialize(bodyPtr, borderPtr->getBody(), borderPtr->getVec()[n]);
		break;
	}
	jointDef.collideConnected = false;
	jointDef.lowerAngle = -15.0f * b2_pi / 180.0f;
	jointDef.upperAngle = 60.0f * b2_pi / 180.0f;
	jointDef.enableLimit = true;
	jointDef.maxMotorTorque = 10.0f;
	jointPtr = (b2DistanceJoint*)worldPtr->CreateJoint(&jointDef);
}

b2Body* Flipper::getBody()
{
	return bodyPtr;
}

void Flipper::render()
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
