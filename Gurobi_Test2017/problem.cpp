//#include "RigidPD.h"
//#include "DefPD.h"
//#include "DefDefPD.h"
//
//#define rN() ((float)rand() / (RAND_MAX + 1)) //random number 0 ~ 1.0
//
//void quick_sort(double *data, int *index, int start, int end) {
//	if (start >= end) {
//		// 원소가 1개인 경우
//		return;
//	}
//
//	int pivot = start;
//	int i = pivot + 1; // 왼쪽 출발 지점 
//	int j = end; // 오른쪽 출발 지점
//	double temp;
//	int tempI;
//
//	while (i <= j) {
//		// 포인터가 엇갈릴때까지 반복
//		while (i <= end && data[i] <= data[pivot]) {
//			i++;
//		}
//		while (j > start && data[j] >= data[pivot]) {
//			j--;
//		}
//
//		if (i > j) {
//			// 엇갈림
//			temp = data[j];
//			data[j] = data[pivot];
//			data[pivot] = temp;
//
//			tempI = index[j];
//			index[j] = index[pivot];
//			index[pivot] = tempI;
//		}
//		else {
//			// i번째와 j번째를 스왑
//			temp = data[i];
//			data[i] = data[j];
//			data[j] = temp;
//
//			tempI = index[i];
//			index[i] = index[j];
//			index[j] = tempI;
//
//		}
//	}
//
//	// 분할 계산
//	quick_sort(data, index, start, j - 1);
//	quick_sort(data, index, j + 1, end);
//
//	return;
//}
//
//void randomTetGeneration(tet& t) {
//
//	for (int i = 0; i < 4; i++) {
//		t.vertex[i] = vec3(rN(), rN(), rN());
//	}
//}
//void randomVertexGeneration(float* v) {
//	//for (int i = 0; i < 4; i++) {
//	v[0] = rN();
//	v[1] = rN();
//	v[2] = rN();
//	//}
//}
//
//int main(int argc, const char *argv[])
//{
//	float v[2][1000][4][3];
//	tet rTet[2]; //rest tetrahedron
//	tet pTet[2]; //deformed pose tetrahedron
//	double minOptValue[3][1000];
//
//	double sortedPD[1000][44];
//	int sortedPDIndex[1000][44];
//
//	int minOptIndex[3][1000] = { -1 };
//	double totalOptTime[3][1000] = { 0.0 };
//	double meanOptTime[3] = { 0.0 };
//	double maxTime[3] = { 0.0 };
//	double minTime[3] = { INFINITY };
//	double comparePercent = 0.0;
//	int metricComparison = 0;
//	int directionCheck = 0;
//	int directionChecks[44] = { 0 };
//	int directionRank[1000] = { 0 };
//	double averageDirectoinRank = 0;
//	RigidPD rigidPD;
//	DefPD defPD;
//	DefDefPD defdefPD;
//	int numTest = 1000;
//	double totalRigidTime = 0;
//	srand(time(NULL));
//
//	string filename1;
//	string filename2;
//
//	time_t totalSec;
//	time(&totalSec);
//	tm *pt = localtime(&totalSec);
//	string fileMadeTime = "_" + to_string(pt->tm_year + 1900) + "_" + to_string(pt->tm_mon + 1) + "_" + to_string(pt->tm_mday) + "_" + to_string(pt->tm_hour) + "_" + to_string(pt->tm_min) + "_" + to_string(pt->tm_sec);
//
//	filename1 = "test" + fileMadeTime + "def";
//	filename2 = "test" + fileMadeTime + "rigid";
//
//	defdefPD.writeCSVHead(filename1);
//	rigidPD.writeCSVHead(filename2);
//
//	int count = 0;
//	for (int i = 0; i < numTest; i++) {
//		//tet generation, volume!=0, penetrated tet pairs;
//		do
//		{
//			for (int j = 0; j < 2; j++) {
//				do
//				{
//					for (int vN = 0; vN < 4; vN++) {
//						randomVertexGeneration(&v[j][i][vN][0]);
//
//						rTet[j].vertex[vN].x = (v[j][i][vN][0]);
//						rTet[j].vertex[vN].y = (v[j][i][vN][1]);
//						rTet[j].vertex[vN].z = (v[j][i][vN][2]);
//					}
//					//randomTetGeneration(rTet[j]);
//				} while (rigidPD.calculateTetVolume(rTet[j]) == 0);
//
//			}
//			rigidPD.init(rTet[0], rTet[1]);
//			//if (rigidPD.resolveRigidPenetration() == false) count++;
//
//		} while (rigidPD.resolveRigidPenetration() == false);//if it is separated, generate tet again.
//	}
//
//	/*Rigid PD calculation*/
//	clock_t start = clock();
//	for (int i = 0; i < numTest; i++) {
//		for (int j = 0; j < 2; j++) {
//			for (int vN = 0; vN < 4; vN++) {
//				rTet[j].vertex[vN].x = (v[j][i][vN][0]);
//				rTet[j].vertex[vN].y = (v[j][i][vN][1]);
//				rTet[j].vertex[vN].z = (v[j][i][vN][2]);
//			}
//		}
//		rigidPD.init(rTet[0], rTet[1]);
//		rigidPD.resolveRigidPenetration();
//	}
//	clock_t end = clock();
//
//	totalRigidTime = (double)((double)end - (double)start) / (double)CLOCKS_PER_SEC;
//
//	for (int i = 0; i < numTest; i++) {
//
//		totalOptTime[0][i] = rigidPD.getOptTime();
//		minOptValue[0][i] = rigidPD.getPD();
//		minOptIndex[0][i] = rigidPD.getMinOptIndex();
//		for (int k = 0; k < 44; k++) {
//			sortedPD[i][k] = rigidPD.getPTetAll().optValue[k];
//			sortedPDIndex[i][k] = k;
//		}
//		quick_sort(&sortedPD[i][0], &sortedPDIndex[i][0], 0, 43);
//		/*for (int k = 0; k < 44; k++) {
//		cout << i << " : [index]=" << sortedPDIndex[i][k] << "[rigid PD]=" << sortedPD[i][k] << endl;
//		}*/
//		//meanOptTime[0] += totalOptTime[0][i];
//
//
//
//		/*defPD.init(rTet[0], rTet[1]);
//		defPD.resolveStaticDefPenetration();
//		totalOptTime[1][i] = defPD.getOptTime();
//		minOptValue[1][i] = defPD.getPD();
//		minOptIndex[1][i] = defPD.getMinOptIndex();
//		*/
//
//		defdefPD.init(rTet[0], rTet[1]);
//		defdefPD.resolveDefDefPenetration();
//		totalOptTime[2][i] = defdefPD.getOptTime();
//		minOptValue[2][i] = defdefPD.getPD();
//		minOptIndex[2][i] = defdefPD.getMinOptIndex();
//
//		for (int j = 0; j < 3; j++) {
//			meanOptTime[j] += totalOptTime[j][i];
//			if (totalOptTime[j][i] < minTime[j]) { minTime[j] = totalOptTime[j][i]; }
//			if (totalOptTime[j][i] > maxTime[j]) { maxTime[j] = totalOptTime[j][i]; }
//		}
//
//		if (minOptValue[0][i] > minOptValue[2][i]) {
//			metricComparison++;
//			comparePercent += (minOptValue[0][i] - minOptValue[2][i]) / minOptValue[0][i];
//		}
//
//		if (minOptIndex[0][i] == minOptIndex[2][i]) {//count if two direction is same
//			directionCheck++;
//			directionRank[i] = 0;
//			directionChecks[0]++;
//		}
//		else {
//			directionRank[i] = 1;
//
//			while (minOptIndex[2][i] != sortedPDIndex[i][directionRank[i]]) {
//				directionRank[i] += 1;;
//
//			}
//
//			averageDirectoinRank += (double)directionRank[i] + 1;
//			//for (int haha = 0; haha < directionRank[i]; haha++) {
//			directionChecks[directionRank[i]]++;
//			//}
//		}
//		/*if (sortedPDIndex[i][0] == (minOptIndex[2][i]) || sortedPDIndex[i][1] == (minOptIndex[2][i]) || sortedPDIndex[i][2] == (minOptIndex[2][i])) {
//		directionCheck3rd++;
//		}*/
//
//		defdefPD.writeCSV(filename1);
//		rigidPD.writeCSV(filename2);
//		if (i % 100 == 0) cout << i << endl;
//	}
//
//
//
//	cout << "total Rigid Time *5000= " << totalRigidTime << endl;
//	cout << "number of separated tet pairs:" << count << endl;
//	comparePercent /= numTest; //10000개로 나누고 100퍼센트로 표시
//	averageDirectoinRank /= numTest;
//	for (int i = 0; i < 3; i++) {
//		meanOptTime[i] /= numTest;
//	}
//	cout << "totalOptTime for Rigid: " << meanOptTime[0] << "seconds" << endl;
//	cout << "totalOptTime for Def: " << meanOptTime[1] << "seconds" << endl;
//	cout << "totalOptTime for DefDef: " << meanOptTime[2] << "seconds" << endl;
//
//	cout << "Min,Max time for Rigid: (" << minTime[0] << "," << maxTime[0] << ") seconds" << endl;
//	cout << "Min,Max time for Def:   (" << minTime[1] << "," << maxTime[1] << ") seconds" << endl;
//	cout << "Min,Max time for DefDef:(" << minTime[2] << "," << maxTime[2] << ") seconds" << endl;
//
//
//	cout << "same direction :" << 100.0*(double)directionCheck / (double)numTest << "%" << endl;
//	double sum = 0;
//	for (int haha = 0; haha < 44; haha++)
//	{
//		double v = 100.0*(double)directionChecks[haha] / (double)numTest;
//		sum += v;
//		cout << "possiblity of within " << haha << "th direction :" << v << "%, " << sum << "%" << endl;
//
//	}
//
//	cout << "averate Rank:" << averageDirectoinRank << "th rigid PD direction is same with deformed PD direction" << endl;
//	cout << "rigid > defdef : " << 100.0*(double)metricComparison / (double)numTest << "%" << endl;
//	cout << "Mean differeces between metric: " << 100.0*comparePercent << "%" << endl;
//
//
//
//	//defdefPD->printResult(minOptIndex);
//
//	for (int i = 0; i < numTest; i++) {
//		cout << "rigid PD [" << i << "]= " << minOptValue[0][i] << "\t" << "DefDef PD [" << i << "]= " << minOptValue[2][i] << endl;
//		cout << "rigid direction [" << i << "]= " << minOptIndex[0][i] << "\t" << "DefDef direction [" << i << "]= " << minOptIndex[2][i] << endl;
//		cout << directionRank[i] << "th value:" << "rigid direction[" << sortedPDIndex[i][directionRank[i]] << "] =" << sortedPD[i][directionRank[i]] << endl;
//		cout << endl;
//	}
//	//1. random generation : if volume==0, regenerate.
//	// 1) Metric values for each cases --> value comparison, how much?
//	// 2) Opt Pair indices for each cases --> calculate percentage of accuracy if not 100%
//	// 3) Total time for each cases --> speed comparison
//	filename1 = "test" + fileMadeTime + "Total Results";
//
//	ofstream output(filename1 + ".txt", ios::app);
//
//	output << "totalOptTime for Rigid: " << meanOptTime[0] << "seconds" << endl;
//	output << "totalOptTime for Def: " << meanOptTime[1] << "seconds" << endl;
//	output << "totalOptTime for DefDef: " << meanOptTime[2] << "seconds" << endl;
//
//	output << "Min,Max time for Rigid: (" << minTime[0] << "," << maxTime[0] << ") seconds" << endl;
//	output << "Min,Max time for Def:   (" << minTime[1] << "," << maxTime[1] << ") seconds" << endl;
//	output << "Min,Max time for DefDef:(" << minTime[2] << "," << maxTime[2] << ") seconds" << endl;
//
//
//	output << "same direction :" << 100.0*(double)directionCheck / (double)numTest << "%" << endl;
//	sum = 0;
//	for (int haha = 0; haha < 44; haha++)
//	{
//		double v = 100.0*(double)directionChecks[haha] / (double)numTest;
//		sum += v;
//		output << "possiblity of within " << haha << "th direction :" << v << "%, " << sum << "%" << endl;
//
//	}
//
//	output << "averate Rank:" << averageDirectoinRank << "th rigid PD direction is same with deformed PD direction" << endl;
//	output << "rigid > defdef : " << 100.0*(double)metricComparison / (double)numTest << "%" << endl;
//	output << "Mean differeces between metric: " << 100.0*comparePercent << "%" << endl;
//
//
//
//	//defdefPD->printResult(minOptIndex);
//
//	for (int i = 0; i < numTest; i++) {
//		output << "rigid PD [" << i << "]= " << minOptValue[0][i] << "\t" << "DefDef PD [" << i << "]= " << minOptValue[2][i] << endl;
//		output << "rigid direction [" << i << "]= " << minOptIndex[0][i] << "\t" << "DefDef direction [" << i << "]= " << minOptIndex[2][i] << endl;
//		output << directionRank[i] << "th value:" << "rigid direction[" << sortedPDIndex[i][directionRank[i]] << "] =" << sortedPD[i][directionRank[i]] << endl;
//		output << endl;
//	}
//
//
//
//	return 0;
//}
//
