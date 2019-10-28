#pragma once

#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <fstream>

#include <GL/glui.h>
#include <glm/glm.hpp>
#include <vector>

using namespace std;
typedef struct {
	int index;
	double x, y, z;
}Node;
typedef struct {
	int index;
	Node *vertex[4];
}Ele;
class TetModel
{
public:
	//constructor
	TetModel();
	TetModel(string fileName);
	~TetModel();

	void initialize(string fileName);
	void readFile(string fileName);
	void writeFile(string fileName);
	void modifyNode(int index, double x, double y, double z); //modify the position of the specific node
	void modifyElem(int index, int vIndex, double x, double y, double z);

	void render();

	void calculateObjNorm();

	//getter and setter
	void setNumNode(int num) { numNode = num; };
	void setNumEle(int num) { numEle = num; };
	void setNodes(vector<Node> n) { nodes = n; };
	void setElems(vector<Ele> el) { elements = el; };

	int getNumNode() { return numNode; };
	int getNumEle() { return numEle; };
	vector<Node> getNodes() { return nodes; };
	vector<Ele> getElem() { return elements; };
	string getFileName() { return fileName; };
	double getObjValue() { return objValue; };

private:
	bool initialized = false;
	int numNode = 0;
	int numEle = 0;
	vector<Node> nodes;
	vector<Ele> elements;

	string fileName="";
	double objValue=0.0;
	vector<int> edgeCount;
	
};
TetModel::TetModel() {
	initialized = false;

}
TetModel::TetModel(string fileName) {
	readFile(fileName);
}
TetModel::~TetModel() {

}
void TetModel::initialize(string fileName) {
	initialized = false;
	nodes.clear();
	elements.clear();
	edgeCount.clear();
	numNode = 0;
	numEle = 0;
	objValue = 0.0;
	this->fileName = fileName;
}

void TetModel::readFile(string fileName) {
	initialized = false;

	string line;
	ifstream in_node(fileName + ".node");
	ifstream in_elem(fileName + ".ele");
	if (in_node.fail()) {
		cout << "[Error] .node file cannot be opened." << endl;
		return;
	}
	if (in_elem.fail()) {
		cout << "[Error] .ele file cannot be opened." << endl;
		return;
	}
	in_node >> numNode;
	in_elem >> numEle;
	cout << "Num Node:" << numNode << endl;
	cout << "Num Elements:" << numEle << endl;
	nodes.resize(numNode);
	elements.resize(numEle);
	edgeCount.resize(numNode*numNode);

	getline(in_node, line);// remove rest of the first line;
	getline(in_elem, line);

	while (!in_node.eof()) {
		Node temp;
		in_node >> temp.index;
		in_node >> temp.x;
		in_node >> temp.y;
		in_node >> temp.z;
		temp.index--;
		int index = temp.index;
		nodes.at(index) = temp;

		//cout << nodes.at(index).index << endl;
		//cout << "( " << nodes.at(index).x << "," << nodes.at(index).y << "," << nodes.at(index).z << ")" << endl;
		if (temp.index == (numNode - 1)) {
			break;
		}
	}
	while (!in_elem.eof()) {
		int index, vIndex[4];// a, b, c, d;
		Ele el;

		in_elem >> index;
		index--;		 // file index starts from 1, while vector index starts from 0
		el.index = index;
		for (int i = 0; i < 4; i++) {
			in_elem >> vIndex[i];
			el.vertex[i] = &nodes.at(vIndex[i] - 1);
		}

		//cout << "( " << vIndex[0] << "," << vIndex[1] << "," << vIndex[2] << "," << vIndex[3] << ")" << endl; //print original file contents


		elements.at(index) = el;

		//count the number of duplicated edge to calculate the object norm.
		for (int i = 0; i < 4; i++) {
			for (int j = i; j < 4; j++) {
				int eI =(vIndex[i]-1)*numNode+(vIndex[j]-1);
				edgeCount.at(eI)++;
			}
		}
		//nodes.at(temp.index) = temp;

		//cout << elements.at(index).index << endl;
		//cout << "( " << elements.at(index).vertex[0].index<< "," << elements.at(index).vertex[1].index << "," << elements.at(index).vertex[2].index << "," << elements.at(index).vertex[3].index<<")" << endl;
		if ((index + 1) == numEle) { break; }
	}

	in_node.close();
	in_elem.close();
	initialized = true;
}
void TetModel::writeFile(string fileName) {
	ofstream output(fileName + ".node");
	output << numNode << " 3 0 0" << endl;

	for (int i = 0; i < numNode; i++) {
		output << i + 1 << nodes.at(i).x << " " << nodes.at(i).y << " " << nodes.at(i).z << endl;
	}

	output.close();
	output.open(fileName + ".ele");
	output << numEle << " 4 0" << endl;
	for (int i = 0; i < numEle; i++) {
		output << i + 1;
		for (int j = 0; j < 4; j++)
			output << " " << elements.at(i).vertex[j]->index;
		output << endl;
	}

}
void TetModel::modifyNode(int index, double x, double y, double z) {
	nodes.at(index).x = x;
	nodes.at(index).y = y;
	nodes.at(index).z = z;
}
void TetModel::modifyElem(int index, int vIndex, double x, double y, double z) {
	(elements.at(index).vertex[vIndex])->x = x;
	(elements.at(index).vertex[vIndex])->y = y;
	(elements.at(index).vertex[vIndex])->z = z;
}
void TetModel::render() {
	cout << "?" << endl;
	glPushMatrix();
	glTranslatef(0, 0, 0);
	glBegin(GL_TRIANGLES);
	for (int t = 0; t < numEle; t++) { //tetrahedron has 4 triangles
									   //float color = ((float)t / (float)numEle);//*255.0;
									   //cout << "color:"<<color << endl;
		glColor4f(0.1f, 0.1f, 0.1f, 0.1f);
		for (int f = 0; f < 4; f++) {
			//cout << "f" << endl;
			for (int v = f; v < (f + 3); v++) {
				int vI = v;
				//cout << "v" << endl;
				if (v > 4) vI = v % 4;;
				glVertex3f((elements.at(t).vertex[vI])->x*2.0f, elements.at(t).vertex[vI]->y*2.0f, elements.at(t).vertex[vI]->z*2.0f);
			}
		}
	}
	glEnd();

	glBegin(GL_LINES);
	glColor4f(0.1f, 0.1f, 0.1f, 0.7f);
	for (int t = 0; t < numEle; t++) {
		for (int f = 0; f < 4; f++) {
			int i[3] = { f % 4,(f + 1) % 4,(f + 2) % 4 };
			glVertex3f(elements.at(t).vertex[i[0]]->x*2.0f, elements.at(t).vertex[i[0]]->y*2.0f, elements.at(t).vertex[i[0]]->z*2.0f);
			glVertex3f(elements.at(t).vertex[i[1]]->x*2.0f, elements.at(t).vertex[i[1]]->y*2.0f, elements.at(t).vertex[i[1]]->z*2.0f);

			glVertex3f(elements.at(t).vertex[i[1]]->x*2.0f, elements.at(t).vertex[i[1]]->y*2.0f, elements.at(t).vertex[i[1]]->z*2.0f);
			glVertex3f(elements.at(t).vertex[i[2]]->x*2.0f, elements.at(t).vertex[i[2]]->y*2.0f, elements.at(t).vertex[i[2]]->z*2.0f);

			glVertex3f(elements.at(t).vertex[i[2]]->x*2.0f, elements.at(t).vertex[i[2]]->y*2.0f, elements.at(t).vertex[i[2]]->z*2.0f);
			glVertex3f(elements.at(t).vertex[i[0]]->x*2.0f, elements.at(t).vertex[i[0]]->y*2.0f, elements.at(t).vertex[i[0]]->z*2.0f);
		}
	}
	glEnd();
	glPopMatrix();
}

void TetModel::calculateObjNorm(){
	double sum;
	if (initialized) {



		objValue = sum / (double)numEle;
	}
	else {
		cout << "[TetModel:: calculate obj norm] No data has been loaded." << endl;
	}
}
