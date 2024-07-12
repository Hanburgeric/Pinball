#include "Box2D/Box2D.h"
#include "GL/freeglut.h"

#include "RotatingCircle.h"

RotatingCircle::RotatingCircle(b2World* worldPtr, const float& x, const float& y, const float& r, b2Body* center) :
	bodyPtr(),
	jointPtr(),
	centerPtr(center),
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
	fixture.density = 1.0e37f;
	fixture.friction = 0.75f;
	fixture.restitution = 0.75f;
	bodyPtr->CreateFixture(&fixture);
	bodyPtr->SetGravityScale(0.0f);

	b2DistanceJointDef jointDef;
	jointDef.Initialize(centerPtr, bodyPtr, centerPtr->GetPosition(), bodyPtr->GetPosition());
	jointPtr = (b2DistanceJoint*)worldPtr->CreateJoint(&jointDef);
}

RotatingCircle::~RotatingCircle()
{
	bodyPtr = nullptr;
	jointPtr = nullptr;
	centerPtr = nullptr;
}

void RotatingCircle::render()
{
	glPushMatrix();
	glTranslatef(bodyPtr->GetPosition().x, bodyPtr->GetPosition().y, 0.0f);
	glRotatef(bodyPtr->GetAngle() * 180.0f / b2_pi, 0.0f, 0.0f, 1.0f);
	glColor3f(0.863f, 0.847f, 0.753f);

	glutSolidSphere(radius, 100, 100);
	glPopMatrix();
}

void RotatingCircle::update()
{
	bodyPtr->SetLinearVelocity(b2Vec2(bodyPtr->GetPosition().y - centerPtr->GetPosition().y, centerPtr->GetPosition().x - bodyPtr->GetPosition().x));
}
