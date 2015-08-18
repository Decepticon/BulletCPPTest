// BulletCPPTest.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "btBulletDynamicsCommon.h"
#include <iostream>
int _tmain(int argc, _TCHAR* argv[]) 
{
	btBroadphaseInterface* broadphase = new btDbvtBroadphase();
	btDefaultCollisionConfiguration* collisionConfig = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfig);
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
	btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfig);

	dynamicsWorld->setGravity(btVector3(0, -10, 0));

	#pragma region GROUND
	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 1);
	btDefaultMotionState* gMS = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
	btRigidBody::btRigidBodyConstructionInfo gRBCI(0, gMS, groundShape, btVector3(0, 0, 0));
	btRigidBody* gRB = new btRigidBody(gRBCI);
	dynamicsWorld->addRigidBody(gRB);
	#pragma endregion

	#pragma region FALLER
	btCollisionShape* fallerShape = new btSphereShape(1);
	btDefaultMotionState* fMS = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 50, 0)));
	
	btScalar mass = 1;
	btVector3 fInertia(0, 0, 0);
	fallerShape->calculateLocalInertia(mass, fInertia);

	btRigidBody::btRigidBodyConstructionInfo fRBCI(mass, fMS, fallerShape, fInertia);
	btRigidBody* fRB = new btRigidBody(fRBCI);
	dynamicsWorld->addRigidBody(fRB);
	#pragma endregion

	for (int i = 0; i < 300; i++) 
	{
		dynamicsWorld->stepSimulation(1 / 60.f, 10);

		btTransform trans;
		fRB->getMotionState()->getWorldTransform(trans);

		std::cout << "sphere height: " << trans.getOrigin().getY() << std::endl;
	}

	#pragma region DISPOSE
	delete dynamicsWorld;
	delete solver;
	delete dispatcher;
	delete collisionConfig;
	delete broadphase;
	#pragma endregion 

	return std::cin.get();
}