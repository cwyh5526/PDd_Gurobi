#pragma once
#include"DataStructure.h"

using namespace std;

class RigidPD
{
public:
	//constructor
	RigidPD(); 
	~RigidPD();

	tet getSTet() { return sTet.toTet(); };
	tet getRTet() { return rTet.toTet(); };
	tet getPTet(int i);
	float distancePointPlane(vec3 x, plane pl);
	float distancePointPlane(vec3 x, vec3 n, vec3 p);

	plane calculateSeparatingPlane(vec3 faceVrtx[3], vec3 vrtx); //separating plane of face
	plane calculateSeparatingPlane(vec3 edge[2][2],  vec3 vrtx[2][2]); //separating plane of edge/edge pair.

	bool calculateRigidPD();
	//void separatingAxisTest(Tetrahedron sTet, Tetrahedron rTet); //테스트 할 때 모든 axis(plane)에 대하여 distance 구할 것.

	void initDefault();
	void init(tet t1,tet t2);
	void printV3(vec3 v);
	void printResult();

	Tetrahedron sTet, rTet;
	plane pl[44]; //separating directions
	float distances[44]; 
	float rigidPD_Value; //smallest distances
	vec3 rigidPD;// 
	int rigidPD_Pair;
	float totalOptTime;
	optResults2 result;
};
// 1. 모든 separating axis 에 대하여 normal을 구한다.
// 2. 모든 separating axis(plane)에 대하여 plane의 반대 방향에 있는 상대 tet의 vertex를 구하고, 그 vertex에서 plane까지의 거리를 구한 후 가장 작은 값을 구한다.
//