#include <iomanip>

#include "RigidPD.h"

RigidPD::RigidPD() {

}
RigidPD::~RigidPD() {

}
//ax+by+cz+d=0 --> d = -n¡¤q
//x:point, n: plane normal vector, d: constant of plane eq.
float RigidPD::distancePointPlane(vec3 p, plane pl) {
	vec3 n = pl.n;
	float d = pl.d;
	float length = sqrt(n.x*n.x+n.y*n.y+n.z*n.z);
	if (length != 0) {
		return (abs(dot(p, n) + d))/length;
	}
	else {
		return NULL; 
	}
}

//x: point, n: plane normal vector, p: point on plane
float RigidPD::distancePointPlane(vec3 p, vec3 n, vec3 q ) {
	float length = sqrt(n.x*n.x + n.y*n.y + n.z*n.z);
	if (length != 0) {
		return (abs(dot(p-q, n))) / length;
	}
	else {
		return NULL;
	}
}

plane RigidPD::calculateSeparatingPlane(vec3 faceVrtx[3], vec3 vrtx) {
	plane pl;	//ax+by+cz+d=0, d= -n¡¤q; where q is a vertex on the plane
	//compute the normal vector and constant of the plane equation for 3 vertices
	vec3 v01 = faceVrtx[1] - faceVrtx[0];
	vec3 v12 = faceVrtx[2] - faceVrtx[1];
	vec3 n = cross(v01, v12);
	if (dot(vrtx - faceVrtx[1], n) > 0) {//if the calculated normal is inward, change it to outward
		n = -n;
	}
	try {
		pl.n = normalize(n);// divide by zero exception could occur
		pl.d = -dot(n, faceVrtx[0]);
		return pl;
	}
	catch (exception e) {
		cout << "exception while normal calculation of face case" << e.what() << endl;
	}
}
plane RigidPD::calculateSeparatingPlane(vec3 edgeVrtx[2][2], vec3 vrtx[2][2]) {
	cout << "calculateSeparatingPlane"  << endl;

	plane pl;
	vec3 e1 = edgeVrtx[0][1] - edgeVrtx[0][0];
	vec3 e2 = edgeVrtx[1][1] - edgeVrtx[1][0];
	vec3 n_cross = cross(e1, e2);
	vec3 n = normalize(n_cross);

	if (dot(n_cross, n_cross) == 0) {
		vec3 AB = edgeVrtx[0][0] - edgeVrtx[1][0];
		vec3 BCdirection = normalize(e1);
		vec3 BH = sqrt(dot(AB, AB) - dot(cross(BCdirection, -AB), cross(BCdirection, -AB)))*BCdirection;
		n = AB + BH;
		n = normalize(n);
		// both normal has to be tested in this case
		//nDecided = false;
		cout << "\n\n\n\n[FASLE]: 0. Parallel Edges \n\n\n" << endl;
		cout << "n: ";
		printV3(n);
	}
	else {
		/*try {
			n = normalize(n);
			printV3(n);
		}
		catch(exception e){
			pl.n = n;
			pl.d = NULL;
			cout << "exception while normal caculation of edge/edge case" << e.what()<<endl;
		}*/
		cout << "1" << endl;
		float nDotv0 = dot(vrtx[0][0] - edgeVrtx[0][0], n);
		float nDotv1 = dot(vrtx[0][1] - edgeVrtx[0][0], n);
		cout << "2" << endl;

		//if ((nDotv0*nDotv1) < 0) { //contact state of two edge would not resolve the penetration. 
		//	if (dot(vrtx[1][0] - edgeVrtx[1][0], n) <= 0) { //in the normal is outward of other tet, then change it to inward of other tet.
		//		n = -n;
		//	}
		//}
		//else
		if (nDotv0 >= 0 && nDotv1 >= 0) { //if both in the same direction of normal, invert the normal direction
			cout << "3" << endl;
			n = -n;
		}
		cout << "4" << endl;

	}
	pl.n = n;
	pl.d = -dot(n,edgeVrtx[0][0]);
	return pl;

}

bool RigidPD::calculateRigidPD() {
	cout << "optimization start" << endl;
	clock_t start = clock();
	//face of sTet
	int index = 0;
	for (index = 0; index < 4; index++) {
		cout << "Face s" <<index<< endl;

		pl[index] = calculateSeparatingPlane(sTet.face[index], sTet.vertex[index]); // get separating plane of eache face
		distances[index]=0;
		float nDotV[4];
		for (int j = 1; j < 4; j++) {
			nDotV[j] = dot(rTet.vertex[j], pl[index].n);
			if ((nDotV[j] < 0) && (abs(nDotV[j]) > distances[index])) {// for the vertex on the opposite side of plane normal, calculate the distance
				distances[index] = abs(nDotV[j]);
			}
		}
		if (distances[index] == 0) {
			//return false; //not penetrated, there exist separating plane;
		}
		result.planePoint[index] = sTet.face[index][0]-distances[index];
	}

	//face of rTet
	

	for (int i = 0, index=3; i < 4; i++) {
		index++;
		cout << "Face r" << index << endl;

		pl[index]= calculateSeparatingPlane(rTet.face[i], rTet.vertex[i]);
		distances[index] = 0;
		float nDotV[4];
		for (int j = 0; j < 4; j++) {
			nDotV[j] = dot(sTet.vertex[j], pl[index].n);
			if ((nDotV[j] < 0) && (abs(nDotV[j]) > distances[index])) {
				distances[index] = abs(nDotV[j]);
			}
		}
		
		if (distances[index] == 0) {
			//return false; //not penetrated, there exist separating plane;
		}
		result.planePoint[index] = rTet.face[i][0] + distances[index];

	}

	for (int s = 0,index=7; s < 6; s++) {
		for (int r = 0; r < 6; r++) {
			cout << "edge s" << s << " r"<<r<< endl;

			index++;
			vec3 contactEdges[2][2] = { { sTet.edge[s][0],sTet.edge[s][1] }, {rTet.edge[r][0],rTet.edge[r][1]} };
			vec3 nonContactEdges[2][2] = { { sTet.edge[5-s][0],sTet.edge[5-s][1] },{ rTet.edge[5-r][0],rTet.edge[5-r][1] } };
		
			pl[index] = calculateSeparatingPlane(contactEdges, nonContactEdges);
			distances[index] = 0;
			float nDotV[4];
			float dis[2] = { 0.f };
			//cout << "11" << endl;

			for (int j = 0; j <4; j++) {
				if (j < 2)
				{
					//cout << "22" << endl;
					cout << "non";
					printV3(nonContactEdges[0][j] - contactEdges[0][0]);
					cout << "n=";
					printV3(pl[index].n);
					nDotV[j] = dot(nonContactEdges[0][j]-contactEdges[0][0], pl[index].n); //positive-->penetrated
					cout << "non dot n=" << nDotV[j] << endl;
					if ((nDotV[j] > 0) && (nDotV[j] > dis[0])) {
						dis[0] = nDotV[j];
						cout <<"dis[0]="<< dis[0]<<endl;
					}
				}
				else {
					//cout << "33" << endl;
					cout << "non";
					printV3(nonContactEdges[1][j-2] - contactEdges[1][0]);
					cout << "n=";
					printV3(pl[index].n);
					nDotV[j] = dot(nonContactEdges[1][j-2] - contactEdges[0][0], pl[index].n);//negative-->penetrated
					cout << "non dot n=" << nDotV[j] << endl;

					if ((nDotV[j] < 0) && (abs(nDotV[j]) > dis[1])) {
						dis[1] = abs(nDotV[j]);
						cout << "dis[1]=" << dis[1] << endl;

					}
				}				
			}
			//cout << "44" << endl;

			distances[index] = dis[0] + dis[1];
			if (distances[index] == 0) {
				
				//return false;
			}
			result.planePoint[index] = sTet.edge[s][0] - distances[index];
			cout << "pl.n:";
			printV3(pl[index].n);
			cout << "distance of " << index << " : " << distances[index]<<"="<<dis[0] <<"+"<<dis[1]<< endl;
			cout << endl;
		}
	}
	
	rigidPD_Value = INFINITY;
	for (int i = 0; i < 44; i++) {
		if (distances[i] < rigidPD_Value) {
			rigidPD_Value = distances[i];
			rigidPD_Pair = i;
		}
	}
	rigidPD = pl[rigidPD_Pair].n;
	
	clock_t end = clock();
	totalOptTime = (float)((float)end - (float)start) / (float)CLOCKS_PER_SEC;
	cout << "optimization end" << endl;
	printResult();
	for (int i = 0; i < 44; i++) {
		result.index[i] = i;
		result.normal[i] = pl[i].n;
		result.pTets1[i] = getPTet(0);
		result.pTets2[i] = getPTet(1);
		//result.optTime[i] = ;
		result.optValue[i] = distances[i];
	}
	return true;

}
void RigidPD::initDefault() {
	//static tetrahedron position
	sTet.initTet(vec3(0.0, 0.0, 0.0),
		vec3(1.0, 0.0, 0.0),
		vec3(0.0, 1.0, 0.0),
		vec3(0.0, 0.0, 1.0));

	//rest pose tetrahedron position
	rTet.initTet(vec3(0.2, 0.2, 0.2),
		vec3(-1.2, 0.2, 0.2),
		vec3(0.2, 1.2, 0.2),
		vec3(0.7, 0.7, 1.2));
	


	rigidPD_Value = INFINITY;
	rigidPD_Pair = -1;
	totalOptTime = 0.f;
	return;
}

void RigidPD::init(tet tet1, tet tet2) {
	//1. Initialize each tetrahedon
	sTet.initTet(tet1);
	rTet.initTet(tet2);

	rigidPD_Value = INFINITY;
	rigidPD_Pair = -1;

	totalOptTime = 0.f;

}
tet RigidPD::getPTet(int k) {
	Tetrahedron pTet;
	vec3 v[4];
	if (k == 0) { 
		for (int i = 0; i < 4; i++) {
			v[i] = sTet.vertex[i] - 0.5f*rigidPD_Value*rigidPD;
		}
	}
	else {
		for (int i = 0; i < 4; i++) {
			v[i] = rTet.vertex[i] + 0.5f*rigidPD_Value*rigidPD;
		}
		
	}
	pTet.initTet(v[0], v[1], v[2], v[3]);
	return pTet.toTet();
}
void RigidPD::printV3(vec3 v) {
	cout << "(" << v.x << "," << v.y << "," << v.z << ")" << endl;
}
void RigidPD::printResult() {
	cout << "[Optimization Result]" << endl;
	cout << "1. Minimum metric value: " << rigidPD_Value << endl;
	cout << "2.Total Optimization time:" << totalOptTime << endl;
	cout << "3. Separating plane from: " << rigidPD_Pair << endl;

	for (int i = 0; i < 44; i++) {
		cout << "pl["<<i<<"].normal=";
		printV3(pl[i].n);
		cout << "distance[" << i << "]= " << distances[i] << endl;
		cout << endl;
	}
}