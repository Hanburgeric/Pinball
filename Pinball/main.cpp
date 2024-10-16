#include "Box2D/Box2D.h"
#include "GL/freeglut.h"
#include <set>
#include <vector>
#include <utility>

#include "Ball.h"
#include "Border.h"
#include "Box.h"
#include "ContactListener.h"
#include "Flipper.h"
#include "Launcher.h"
#include "MovingPlatform.h"
#include "Pin.h"
#include "Portal.h"
#include "RotatingCircle.h"
#include "RotatingPlatform.h"
#include "Water.h"

//Global constants
const float WINDOW_WIDTH = 540.0f;
const float WINDOW_HEIGHT = 720.0f;

const float FPS = 30.0f;
const float TIME_STEP = 1.0f / FPS;
const int VELOCITY_ITERATIONS = 8;
const int POSITION_ITERATIONS = 3;

const float GRAVITY = -9.81f;

//Contact Listener
ContactListener* contactListenerPtr;

//World
b2World* worldPtr = nullptr;

//Water
Water* waterPtr = nullptr;

//Map
Border* mapPtr = nullptr;

//Flippers
std::vector<Flipper> flipperVec;

//Bumpers
std::vector<Border> bumperVec;

//Pins
std::vector<Pin> pinVec;

//Portals
std::vector<Portal> portalVec;

//Rotating Circles
std::vector<RotatingCircle> rotCircVec;

//Moving Platforms
std::vector<MovingPlatform> movPlatVec;

//Rotating Platforms
std::vector<RotatingPlatform>rotPlatVec;

//Launcher
Launcher* launcherPtr = nullptr;

//Boxes
std::vector<Box> boxVec;

//Ball
Ball* ballPtr = nullptr;

//Callback functions
void init();
void input(unsigned char key, int x, int y);
void update(int value);
void render();
void reshape(int newWidth, int newHeight);

//Finds the vertices of intersection for two polygons
bool isInside(b2Vec2 cp1, b2Vec2 cp2, b2Vec2 p);
b2Vec2 intersection(b2Vec2 cp1, b2Vec2 cp2, b2Vec2 s, b2Vec2 e);
bool findIntersectionofFixtures(b2Fixture* fixtureA, b2Fixture* fixtureB, std::vector<b2Vec2>& outputVertices);

//Applies buoyancy and drag to a polygon intersecting another polygon
b2Vec2 computeCentroid(std::vector<b2Vec2> vec, float& area);
void applyBuoyancy(b2Fixture* obj, b2Fixture* fluid, float area, b2Vec2 gravity, b2Vec2 centroid);
void applyDrag(b2Fixture* obj, b2Fixture* fluid, float area, b2Vec2 centroid);
void applyDragLift(b2Fixture* obj, b2Fixture* fluid, std::vector<b2Vec2>& intersectionPoints);

int main(int argc, char* argv[])
{
	//Initializes glut with the window positioned at the center of the screen.
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
	glutInitWindowSize((int)WINDOW_WIDTH, (int)WINDOW_HEIGHT);
	glutInitWindowPosition((glutGet(GLUT_SCREEN_WIDTH) - (int)WINDOW_WIDTH) / 2, (glutGet(GLUT_SCREEN_HEIGHT) - (int)WINDOW_HEIGHT) / 2);
	glutCreateWindow("Pinball");

	//Initializes everything
	init();

	//Main loop
	glutKeyboardFunc(input);
	glutTimerFunc(20, update, 0);
	glutDisplayFunc(render);
	glutReshapeFunc(reshape);
	glutMainLoop();

	//Cleanup
	delete contactListenerPtr;
	contactListenerPtr = nullptr;

	delete worldPtr;
	worldPtr = nullptr;

	delete waterPtr;
	waterPtr = nullptr;

	delete mapPtr;
	mapPtr = nullptr;

	delete launcherPtr;
	launcherPtr = nullptr;

	delete ballPtr;
	ballPtr = nullptr;

	return 0;
}

void init()
{
	//Initializes the contact listener
	contactListenerPtr = new ContactListener;

	//Initializes the world
	worldPtr = new b2World(b2Vec2(0.0f, GRAVITY));
	worldPtr->SetContactListener(contactListenerPtr);

	//Initializes the water
	waterPtr = new Water(worldPtr, 27.0f, 9.0f, 15.0f, 3.0f);

	//Initializes the map
	mapPtr = new Border(Chain);
	mapPtr->addCurve(b2Vec2(25.5f, 60.0f), b2Vec2(31.5f, 60.0f), -90.0f, 18);
	mapPtr->addCurve(b2Vec2(42.0f, 66.0f), b2Vec2(42.0f, 60.0f), -90.0f, 18);
	mapPtr->addVertex(48.0f, 25.0f);
	mapPtr->addVertex(45.0f, 25.0f);
	mapPtr->addCurve(b2Vec2(45.0f, 60.0f), b2Vec2(42.0f, 60.0f), 90.0f, 18);
	mapPtr->addCurve(b2Vec2(31.5f, 63.0f), b2Vec2(31.5f, 60.0f), 90.0f, 18);
	mapPtr->addVertex(28.5f, 60.0f);
	mapPtr->addVertex(42.0f, 60.0f);
	mapPtr->addVertex(42.0f, 25.0f);
	mapPtr->addVertex(31.5f, 17.5f);	//Right flipper anchor point
	int rightFlipperIndex = mapPtr->getVec().size() - 1;
	mapPtr->addVertex(42.0f, 21.0f);
	mapPtr->addVertex(42.0f, 6.0f);
	mapPtr->addVertex(12.0f, 6.0f);
	mapPtr->addVertex(12.0f, 21.0f);
	mapPtr->addVertex(22.5f, 17.5f);	//Left flipper anchor point
	int leftFlipperIndex = mapPtr->getVec().size() - 1;
	mapPtr->addVertex(12.0f, 25.0f);
	mapPtr->addVertex(12.0f, 60.0f);
	mapPtr->addVertex(25.5f, 60.0f);
	mapPtr->createBorder(worldPtr);

	//Initializes the flippers
	const float flipperHalfL = 1.75f;
	const float flipperHalfW = 0.35f;

	flipperVec.push_back(Flipper(worldPtr, Left,
		mapPtr->getVec()[leftFlipperIndex].x + flipperHalfL, mapPtr->getVec()[leftFlipperIndex].y - flipperHalfW,
		flipperHalfL, flipperHalfW));
	flipperVec[Left].createJoint(worldPtr, mapPtr, leftFlipperIndex);

	flipperVec.push_back(Flipper(worldPtr, Right,
		mapPtr->getVec()[rightFlipperIndex].x - flipperHalfL, mapPtr->getVec()[rightFlipperIndex].y - flipperHalfW,
		flipperHalfL, flipperHalfW));
	flipperVec[Right].createJoint(worldPtr, mapPtr, rightFlipperIndex);

	//Initializes the bumpers
	bumperVec.push_back(Border(Loop));
	bumperVec[0].addVertex(15.5f, 32.5f);
	bumperVec[0].addVertex(15.5f, 25.0f);
	bumperVec[0].addVertex(22.5f, 20.0f);
	bumperVec[0].createBorder(worldPtr);

	bumperVec.push_back(Border(Loop));
	bumperVec[1].addVertex(38.5f, 32.5f);
	bumperVec[1].addVertex(31.5f, 20.0f);
	bumperVec[1].addVertex(38.5f, 25.0f);
	bumperVec[1].createBorder(worldPtr);

	//Initializes the pins
	pinVec.push_back(Pin(worldPtr, 21.0f, 58.0f, 0.25f));
	pinVec.push_back(Pin(worldPtr, 33.0f, 58.0f, 0.25f));
	pinVec.push_back(Pin(worldPtr, 22.5f, 56.0f, 0.25f));
	pinVec.push_back(Pin(worldPtr, 31.5f, 56.0f, 0.25f));
	pinVec.push_back(Pin(worldPtr, 24.0f, 54.0f, 0.25f));
	pinVec.push_back(Pin(worldPtr, 30.0f, 54.0f, 0.25f));
	pinVec.push_back(Pin(worldPtr, 25.5f, 52.0f, 0.25f));
	pinVec.push_back(Pin(worldPtr, 28.5f, 52.0f, 0.25f));
	pinVec.push_back(Pin(worldPtr, 27.0f, 50.0f, 0.25f));

	//Initializes the portals
	portalVec.push_back(Portal(worldPtr, Entry, 20.0f, 38.0f, 2.5f));
	portalVec.push_back(Portal(worldPtr, Exit, 37.5f, 55.5f, 2.5f));

	//Initializes the rotating circles
	b2Body* center = portalVec[0].getBody();
	rotCircVec.push_back(RotatingCircle(worldPtr, 23.0f, 41.0f, 1.5f, center));
	rotCircVec.push_back(RotatingCircle(worldPtr, 17.0f, 41.0f, 1.5f, center));
	rotCircVec.push_back(RotatingCircle(worldPtr, 17.0f, 35.0f, 1.5f, center));
	rotCircVec.push_back(RotatingCircle(worldPtr, 23.0f, 35.0f, 1.5f, center));

	//Initializes the moving platforms
	movPlatVec.push_back(MovingPlatform(worldPtr, 14.0f, 54.0f, 0.25f, 1.5f));
	movPlatVec.push_back(MovingPlatform(worldPtr, 20.0f, 50.0f, 0.25f, 1.5f));

	//Initializes the rotating platforms
	rotPlatVec.push_back(RotatingPlatform(worldPtr, 38.0f, 44.0f, 1.5f, 0.25f));
	rotPlatVec.push_back(RotatingPlatform(worldPtr, 34.0f, 48.0f, 0.25f, 1.5f));
	rotPlatVec.push_back(RotatingPlatform(worldPtr, 30.0f, 44.0f, 1.5f, 0.25f));
	rotPlatVec.push_back(RotatingPlatform(worldPtr, 34.0f, 40.0f, 0.25f, 1.5f));

	//Initializes the launcher 42.5
	launcherPtr = new Launcher(worldPtr, 46.5f, 52.5f, 1.45f, 2.9f);
	launcherPtr->createBodyAndJoint(worldPtr, 46.5f, 25.0f);

	//Initializes the boxes
	boxVec.push_back(Box(worldPtr, 15.0f, 14.5f, 2.0f, 0.5f));
	boxVec.push_back(Box(worldPtr, 27.0f, 15.0f, 1.0f, 1.0f));
	boxVec.push_back(Box(worldPtr, 39.0f, 16.0f, 0.5f, 2.0f));

	//Initializes the ball 46.5
	ballPtr = new Ball(worldPtr, 46.5f, 56.5f, 0.75f);
}

void input(unsigned char key, int x, int y)
{
	switch (key)
	{
		//LATER IMPROVEMENT: Find a smoother way to apply forces
	case 32:	//Spacebar
		launcherPtr->getBody()->ApplyLinearImpulseToCenter(b2Vec2(0.0f, -125.0f), true);
		break;
	case 'z':
		flipperVec[Left].getBody()->ApplyTorque(125000.0, true);
		break;
	case '/':
		flipperVec[Right].getBody()->ApplyTorque(-125000.0f, true);
		break;
	}
}

void update(int value)
{
	worldPtr->Step(TIME_STEP, VELOCITY_ITERATIONS, POSITION_ITERATIONS);

	if (contactListenerPtr->fixturePtrPairs.size() > 0)
	{
		std::set<std::pair<b2Fixture*, b2Fixture*>>::iterator iter = contactListenerPtr->fixturePtrPairs.begin();
		std::set<std::pair<b2Fixture*, b2Fixture*>>::iterator end = contactListenerPtr->fixturePtrPairs.end();

		while (iter != end)
		{
			b2Fixture* dynamicFixture = iter->first;
			b2Fixture* sensorFixture = iter->second;
			
			//If the ball is fully inside the entry portal, it is moved to the exit portal.
			if (dynamicFixture->GetShape()->GetType() == b2Shape::e_circle && sensorFixture->GetShape()->GetType() == b2Shape::e_circle)
			{
				if (sensorFixture->GetBody()->GetPosition() == portalVec[Entry].getBody()->GetPosition())
				{
					if ((sensorFixture->GetBody()->GetPosition() - ballPtr->getBody()->GetPosition()).Normalize()
						<= portalVec[Entry].getRadius() - ballPtr->getRadius())
					{
						ballPtr->getBody()->SetTransform(portalVec[Exit].getBody()->GetPosition(), ballPtr->getBody()->GetAngle());
					}
				}
			}

			//If the boxes or the ball collides with the water, buoyancy, drag, and lift forces are applied.
			else
			{
				float density = sensorFixture->GetDensity();

				std::vector<b2Vec2> intersectionPoints;
				if (findIntersectionofFixtures(dynamicFixture, sensorFixture, intersectionPoints))
				{
					float area = 0.0f;
					b2Vec2 centroid = computeCentroid(intersectionPoints, area);

					applyBuoyancy(dynamicFixture, sensorFixture, area, worldPtr->GetGravity(), centroid);
					//applyDrag(ballFixture, waterFixture, area, centroid);
					applyDragLift(dynamicFixture, sensorFixture, intersectionPoints);
				}
			}
			iter++;
		}
	}

	//Calculates and applies the (clockwise) tangent linear force to the rotating circles
	for (int index = 0; index < rotCircVec.size(); index++)
	{
		rotCircVec[index].update();
	}

	//Moves the moving platforms by keeping track of their positions
	for (int index = 0; index < movPlatVec.size(); index++)
	{
		movPlatVec[index].update();
	}

	glutPostRedisplay();
	glutTimerFunc(20, update, 0);
}


void render()
{
	//Initializes gl
	glClearColor(0.25f, 0.25f, 0.25f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//Renders the water
	waterPtr->render();

	//Renders the map
	mapPtr->render();

	//Renders the flippers
	for (int index = 0; index < flipperVec.size(); index++)
	{
		flipperVec[index].render();
	}

	//Renders the bumpers
	for (int index = 0; index < bumperVec.size(); index++)
	{
		bumperVec[index].render();
	}

	//Renders the pins
	for (int index = 0; index < pinVec.size(); index++)
	{
		pinVec[index].render();
	}

	//Renders the portals
	for (int index = 0; index < portalVec.size(); index++)
	{
		portalVec[index].render();
	}

	//Renders the rotating circles
	for (int index = 0; index < rotCircVec.size(); index++)
	{
		rotCircVec[index].render();
	}

	//Renders the moving platforms
	for (int index = 0; index < movPlatVec.size(); index++)
	{
		movPlatVec[index].render();
	}

	//Renders the rotating platforms
	for (int index = 0; index < rotPlatVec.size(); index++)
	{
		rotPlatVec[index].render();
	}

	//Renders the launcher
	launcherPtr->render();

	//Renders the boxes
	for (int index = 0; index < boxVec.size(); index++)
	{
		boxVec[index].render();
	}

	//Renders the ball
	ballPtr->render();

	//Swaps buffers
	glutSwapBuffers();
}

void reshape(int newWidth, int newHeight)
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0.0f, (float)newWidth / 10.0f, 0.0f, (float)newHeight / 10.0f);
	glViewport(0, 0, newWidth, newHeight);
	glutPostRedisplay();
}

bool isInside(b2Vec2 cp1, b2Vec2 cp2, b2Vec2 p)
{
	return (cp2.x - cp1.x) * (p.y - cp1.y) > (cp2.y - cp1.y) * (p.x - cp1.x);
}

b2Vec2 intersection(b2Vec2 cp1, b2Vec2 cp2, b2Vec2 s, b2Vec2 e)
{
	b2Vec2 dc(cp1.x - cp2.x, cp1.y - cp2.y);
	b2Vec2 dp(s.x - e.x, s.y - e.y);
	float n1 = cp1.x * cp2.y - cp1.y * cp2.x;
	float n2 = s.x * e.y - s.y * e.x;
	float n3 = 1.0f / (dc.x * dp.y - dc.y * dp.x);
	return b2Vec2((n1 * dp.x - n2 * dc.x) * n3, (n1 * dp.y - n2 * dc.y) * n3);
}

bool findIntersectionofFixtures(b2Fixture* fixtureA, b2Fixture* fixtureB, std::vector<b2Vec2>& outputVertices)
{
	std::vector<b2Vec2> clipPolygon;

	if (fixtureA->GetShape()->GetType() == b2Shape::e_polygon && fixtureB->GetShape()->GetType() == b2Shape::e_polygon)
	{
		b2PolygonShape* polyShapeA = (b2PolygonShape*)fixtureA->GetShape();
		for (int index = 0; index < polyShapeA->m_count; index++)
		{
			outputVertices.push_back(fixtureA->GetBody()->GetWorldPoint(polyShapeA->m_vertices[index]));
		}

		b2PolygonShape* polyShapeB = (b2PolygonShape*)fixtureB->GetShape();
		for (int index = 0; index < polyShapeB->m_count; index++)
		{
			clipPolygon.push_back(fixtureB->GetBody()->GetWorldPoint(polyShapeB->m_vertices[index]));
		}
	}

	else if (fixtureA->GetShape()->GetType() == b2Shape::e_circle && fixtureB->GetShape()->GetType() == b2Shape::e_polygon)
	{
		b2CircleShape* circShapeA = (b2CircleShape*)fixtureA->GetShape();
		int numEdge = 73;
		float deltaAngle = 2.0f * b2_pi / (float)numEdge;
		for (int index = 0; index < numEdge; index++)
		{
			float x = fixtureA->GetBody()->GetPosition().x + circShapeA->m_radius * cos(fixtureA->GetBody()->GetAngle() + (float)index * deltaAngle);
			float y = fixtureA->GetBody()->GetPosition().y + circShapeA->m_radius * sin(fixtureA->GetBody()->GetAngle() + (float)index * deltaAngle);
			outputVertices.push_back(b2Vec2(x, y));
		}

		b2PolygonShape* polyShapeB = (b2PolygonShape*)fixtureB->GetShape();
		for (int index = 0; index < polyShapeB->m_count; index++)
		{
			clipPolygon.push_back(fixtureB->GetBody()->GetWorldPoint(polyShapeB->m_vertices[index]));
		}
	}

	else
	{
		return false;
	}

	b2Vec2 cp1 = clipPolygon[clipPolygon.size() - 1];
	for (int j = 0; j < clipPolygon.size(); j++)
	{
		if (outputVertices.empty())
		{
			return false;
		}

		else
		{
			b2Vec2 cp2 = clipPolygon[j];
			std::vector<b2Vec2> inputList = outputVertices;
			outputVertices.clear();

			b2Vec2 s = inputList[inputList.size() - 1];
			for (int i = 0; i < inputList.size(); i++)
			{
				b2Vec2 e = inputList[i];

				if (isInside(cp1, cp2, e))
				{
					if (!isInside(cp1, cp2, s))
					{
						outputVertices.push_back(intersection(cp1, cp2, s, e));
					}

					outputVertices.push_back(e);
				}

				else if (isInside(cp1, cp2, s))
				{
					outputVertices.push_back(intersection(cp1, cp2, s, e));
				}

				s = e;
			}

			cp1 = cp2;
		}
	}

	return !outputVertices.empty();
}

b2Vec2 computeCentroid(std::vector<b2Vec2> vec, float& area)
{
	int count = (int)vec.size();
	b2Assert(count >= 3);

	b2Vec2 c(0.0f, 0.0f);
	area = 0.0f;

	b2Vec2 ref(0.0f, 0.0f);
	const float inv3 = 1.0f / 3.0f;

	for (int i = 0; i < count; i++)
	{
		b2Vec2 p1 = ref;
		b2Vec2 p2 = vec[i];
		b2Vec2 p3 = i + 1 < count ? vec[i + 1] : vec[0];

		b2Vec2 e1 = p2 - p1;
		b2Vec2 e2 = p3 - p1;

		float d = b2Cross(e1, e2);
		float triArea = 0.5f * d;
		area += triArea;

		c += triArea * inv3 * (p1 + p2 + p3);
	}

	if (area > b2_epsilon)
	{
		c *= 1.0f / area;
	}

	else
	{
		area = 0.0f;
	}

	return c;
}

void applyBuoyancy(b2Fixture* obj, b2Fixture* fluid, float area, b2Vec2 gravity, b2Vec2 centroid)
{
	float displacedMass = fluid->GetDensity() * area;
	b2Vec2 buoyancyForce = displacedMass * -gravity;
	obj->GetBody()->ApplyForce(buoyancyForce, centroid, true);
}

void applyDrag(b2Fixture* obj, b2Fixture* fluid, float area, b2Vec2 centroid)
{
	b2Vec2 velDirection = obj->GetBody()->GetLinearVelocityFromWorldPoint(centroid) - fluid->GetBody()->GetLinearVelocityFromWorldPoint(centroid);
	float vel = velDirection.Normalize();

	float linDrag = fluid->GetDensity() * vel * vel / 2.0f;
	b2Vec2 dragForce = linDrag * -velDirection;
	obj->GetBody()->ApplyForce(dragForce, centroid, true);

	float angDrag = area * -fluid->GetBody()->GetAngularVelocity();
	obj->GetBody()->ApplyTorque(angDrag, true);
}

void applyDragLift(b2Fixture* obj, b2Fixture* fluid, std::vector<b2Vec2>& intersectionPoints)
{
	for (int index = 0; index < intersectionPoints.size(); index++)
	{
		b2Vec2 p0 = intersectionPoints[index];
		b2Vec2 p1 = intersectionPoints[(index + 1) % intersectionPoints.size()];
		b2Vec2 midpoint = 0.5f * (p0 + p1);

		//Finds the relative velocity between the edge and the fluid
		b2Vec2 velDirection = obj->GetBody()->GetLinearVelocityFromWorldPoint(midpoint) - fluid->GetBody()->GetLinearVelocityFromWorldPoint(midpoint);
		float vel = velDirection.Normalize();

		//Calculates the normal vector
		b2Vec2 edge = p1 - p0;
		float edgeLength = edge.Normalize();
		b2Vec2 normalVector = b2Cross(-1.0f, edge);

		//Skips the edge if the normal points away from the direction of movement
		float dragDot = b2Dot(normalVector, velDirection);
		if (dragDot < 0)
		{
			continue;
		}

		else
		{
			float dragMag = dragDot * edgeLength * fluid->GetDensity() * vel * vel;
			obj->GetBody()->ApplyForce(dragMag * -velDirection, midpoint, true);
		}

		float liftDot = b2Dot(edge, velDirection);
		float liftMag = (dragDot * liftDot) * edgeLength * fluid->GetDensity() * vel * vel;
		b2Vec2 liftDirection = b2Cross(1.0f, velDirection);
		b2Vec2 liftForce = liftMag * liftDirection;
		obj->GetBody()->ApplyForce(liftForce, midpoint, true);
	}
}
