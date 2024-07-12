#include "Box2D/Box2D.h"
#include "GL/freeglut.h"

#include "Border.h"

Border::Border(const BorderType& type) :
	bodyPtr(),
	borderType(type)
{
}

Border::~Border()
{
	bodyPtr = nullptr;
}

void Border::addVertex(const float& x, const float& y)
{
	vertices.push_back(b2Vec2(x, y));
}

void Border::addCurve(const b2Vec2& start, const b2Vec2& center, const float& deg, const int& n)
{
	float deltaAngle = deg * b2_pi / 180.0f / (float)n;

	for (int index = 0; index <= n; index++)
	{
		float cosResult = cos((float)index * deltaAngle);
		float sinResult = sin((float)index * deltaAngle);
		float x = center.x + cosResult * (start.x - center.x) - sinResult * (start.y - center.y);
		float y = center.y + sinResult * (start.x - center.x) + cosResult * (start.y - center.y);
		vertices.push_back(b2Vec2(x, y));
	}
}

void Border::createBorder(b2World* worldPtr)
{
	b2BodyDef bodyDef;
	bodyPtr = worldPtr->CreateBody(&bodyDef);

	b2ChainShape shape;
	b2Vec2* tempAry = new b2Vec2[vertices.size()];
	for (int index = 0; index < vertices.size(); index++)
	{
		tempAry[index] = vertices[index];
	}
	switch (borderType)
	{
	case Chain:
		shape.CreateChain(tempAry, vertices.size());
		break;
	case Loop:
		shape.CreateLoop(tempAry, vertices.size());
		break;
	}
	delete[] tempAry;
	tempAry = nullptr;


	b2FixtureDef fixture;
	fixture.shape = &shape;
	fixture.density = 0.75f;
	fixture.friction = 0.75f;
	fixture.restitution = 0.75f;
	bodyPtr->CreateFixture(&fixture);
}

b2Body* Border::getBody()
{
	return bodyPtr;
}

std::vector<b2Vec2> Border::getVec()
{
	return vertices;
}

void Border::render()
{
	glPushMatrix();
	glTranslatef(bodyPtr->GetPosition().x, bodyPtr->GetPosition().y, 0.0f);
	glRotatef(bodyPtr->GetAngle() * 180.0f / b2_pi, 0.0f, 0.0f, 1.0f);
	glColor3f(0.0f, 0.0f, 0.0f);
	glLineWidth(2.0f);

	switch (borderType)
	{
	case Chain:
		glBegin(GL_LINE_STRIP);
		break;
	case Loop:
		glBegin(GL_LINE_LOOP);
		break;
	}
	for (int index = 0; index < vertices.size(); index++)
	{
		glVertex2f(vertices[index].x, vertices[index].y);
	}
	glEnd();
	glPopMatrix();
}
