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
	//void separatingAxisTest(Tetrahedron sTet, Tetrahedron rTet); //�׽�Ʈ �� �� ��� axis(plane)�� ���Ͽ� distance ���� ��.

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
// 1. ��� separating axis �� ���Ͽ� normal�� ���Ѵ�.
// 2. ��� separating axis(plane)�� ���Ͽ� plane�� �ݴ� ���⿡ �ִ� ��� tet�� vertex�� ���ϰ�, �� vertex���� plane������ �Ÿ��� ���� �� ���� ���� ���� ���Ѵ�.
//