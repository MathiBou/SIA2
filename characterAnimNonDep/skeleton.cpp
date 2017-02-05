#include "skeleton.h"
#include <skeletonIO.h>
#include <qglviewer.h>
#include <iostream>
#include <fstream>

using namespace std;

Skeleton* Skeleton::createFromFile(std::string fileName) {
	Skeleton* root = NULL;
	cout << "Loading from " << fileName << endl;

	ifstream inputfile(fileName.data());
	if (inputfile.good()) {
		while (!inputfile.eof()) {
			string buf;
			inputfile >> buf;
			if (!buf.compare("HIERARCHY")) {
				root = readHierarchy(inputfile);
			}
		}
		inputfile.close();
	}
	else {
		std::cerr << "Failed to load the file " << fileName.data() << std::endl;
		fflush(stdout);
	}

	cout << "file loaded" << endl;

	return root;
}

Skeleton* Skeleton::interpolateFromFiles(std::string file1, std::string file2) {
	Skeleton* root = NULL;
	cout << "Interpolation between" << file1 << "  and  " << file2 << endl;

	ifstream inputfile1(file1.data());
	ifstream inputfile2(file2.data());

	if (inputfile1.good() && inputfile2.good()) {
		while (!inputfile1.eof() && !inputfile2.eof()) {
			string buf;
			inputfile1 >> buf;
			inputfile2 >> buf;

			if (!buf.compare("HIERARCHY")) {
				root = interpolate(inputfile1, inputfile2);
			}
		}
		inputfile1.close();
		inputfile2.close();

	}

	return root;
}

Skeleton* Skeleton::transitionFromFiles(std::string file1, int lastframe1, std::string file2, int firstframe2, float interpolationValue) {

	Skeleton* root = NULL;
	cout << "Transition" << file1 << "  and  " << file2 << endl;

	ifstream inputfile1(file1.data());
	ifstream inputfile2(file2.data());

	if (inputfile1.good() && inputfile2.good()) {
		while (!inputfile1.eof() && !inputfile2.eof()) {
			string buf;
			inputfile1 >> buf;
			inputfile2 >> buf;

			if (!buf.compare("HIERARCHY")) {
				root = transition(inputfile1, inputfile2);
			}
		}
		inputfile1.close();
		inputfile2.close();

		cout << "end set pose" << endl;
	}

	return root;
}


Skeleton* Skeleton::transitionQuaternions(Skeleton* root, Skeleton* second, int nbTransitionFrames) {
	
	/*Skeleton* root = create("hip", first->_offX, first->_offY, first->_offZ, NULL); 
	root->_children = first->_children; 
	root->_dofs = first->_dofs; 
	for ((Skeleton* child : root->_children) && (Skeleton* childF : first->_children)) {
		child = create(first->_name, first->_offX, first->_offY, first->_offZ, NULL);
		root->_children = first->_children;
		root->_dofs = first->_dofs;
	}*/

	/*Skeleton* root = first;*/

	int nbFirst = root->_dofs.at(0)._values.size(); 
	std::cout << "nb frames so far" << root->_dofs.at(0)._values.size() << endl ;

	std::cout << "nb frames run" << second->_dofs.at(0)._values.size() << endl;
	
	double lastRX, lastRY, lastRZ;
	lastRX = second->_dofs.at(3)._values.at(0);
	lastRY = second->_dofs.at(4)._values.at(0);
	lastRZ = second->_dofs.at(5)._values.at(0);

	double lastTX, lastTY, lastTZ;
	lastTX = second->_dofs.at(0)._values.at(0);
	lastTY = second->_dofs.at(1)._values.at(0);
	lastTZ = second->_dofs.at(2)._values.at(0);

	cout << "LAST " << lastTX << " " << lastTY << " " << lastTZ << " " << lastRX << " " << lastRY << " " << lastRZ << endl;
	loop(root, second, nbTransitionFrames, lastTX, lastTY, lastTZ, lastRX, lastRY, lastRZ);

	std::cout << "NB frames in root : " << root->_dofs.at(0)._values.size() << endl;

	updateTransitionFrames(nbFirst, nbTransitionFrames, root);

	return root;
}

void Skeleton::loop(Skeleton* root, Skeleton* second, int nbTransitionFrames, double TX, double TY, double TZ, double RX, double RY, double RZ) {
	static ofstream file("loop.txt", ios::out | ios::trunc);
	
	file << "loop " << root->_name << endl;
	double offset;
	double firstValue;
	bool isRoot = (root->_name == "hip"); 


	for (int i = 0; i < root->_dofs.size(); i++) {
		//Add empty transition frames
		for (int j = 0; j < nbTransitionFrames; j++) {
			root->_dofs.at(i)._values.push_back(0);
		}
		//Add values for second anim
		file << "nb frames in run : " << second->_dofs.at(0)._values.size() << endl;
		for (int k = 0; k < second->_dofs.at(0)._values.size(); k++) {
			if (k == 0) {
				firstValue = second->_dofs.at(i)._values.at(0);
				file << root->_name << "  i : " << i << endl;
				file << "firstValue " << firstValue << endl ; 
				}

			if (isRoot) {
				file << "loop IS ROOT" << endl; 
				switch (i){
				case 0: {
					file << "case 0" << endl;
					offset = TX;
				}
				case 1: {
					offset = TY;
				}
				case 2: {
					offset = TZ;
				}
				case 3: {
					offset = RX;
				}
				case 4: {
					offset = RY;
				}
				case 5: {
					offset = RZ;
				}
				}
			}
			else {
				file << "loop not ROOT" << endl;

				switch (i){
				case 0: {
					offset = RX;
				}
				case 1: {
					offset = RY;
				}
				case 2: {
					offset = RZ;
				}
				}
			}

			file << "What joint : " << root->_name << endl;
			file << "Dof i :" << i << endl;
			file << "Offset :" << offset << endl;
			file << "First value :" << firstValue << endl;

			root->_dofs.at(i)._values.push_back( offset + (second->_dofs.at(i)._values.at(k) - firstValue));
		}
	}

}





void drawBone(Skeleton *child)
{
	qglviewer::Vec v0(0, 0, 1);
	qglviewer::Vec v1(child->_offX, child->_offY, child->_offZ);
	qglviewer::Vec vRot = v0^v1; vRot.normalize();
	float angle = acosf((v0*v1) / (v0.norm()*v1.norm()))*180.0 / M_PI;
	float height = (v1 - v0).norm();
	float radius = 0.1f;
	glPushMatrix();
	{
		glRotatef(angle, vRot.x, vRot.y, vRot.z);
		gluCylinder(gluNewQuadric(), 0.1, 0.1, height, 5, 5);
	}
	glPopMatrix();

}

void Skeleton::rotateSkeleton() {
	switch (_rorder) {
	case roXYZ:
		glRotatef(_curRx, 1, 0, 0);
		glRotatef(_curRy, 0, 1, 0);
		glRotatef(_curRz, 0, 0, 1);
		break;
	case roYZX:
		glRotatef(_curRy, 0, 1, 0);
		glRotatef(_curRz, 0, 0, 1);
		glRotatef(_curRx, 1, 0, 0);
		break;
	case roZXY:
		glRotatef(_curRz, 0, 0, 1);
		glRotatef(_curRx, 1, 0, 0);
		glRotatef(_curRy, 0, 1, 0);
		break;
	case roXZY:
		glRotatef(_curRx, 1, 0, 0);
		glRotatef(_curRz, 0, 0, 1);
		glRotatef(_curRy, 0, 1, 0);
		break;
	case roYXZ:
		glRotatef(_curRy, 0, 1, 0);
		glRotatef(_curRx, 1, 0, 0);
		glRotatef(_curRz, 0, 0, 1);
		break;
	case roZYX:
		glRotatef(_curRz, 0, 0, 1);
		glRotatef(_curRy, 0, 1, 0);
		glRotatef(_curRx, 1, 0, 0);
		break;
	}
}
void Skeleton::draw()
{
	glPushMatrix();
	{
		// Set good reference frame :
		glTranslatef(_offX, _offY, _offZ);
		// Use current value of dofs :
		glTranslatef(_curTx, _curTy, _curTz);
		rotateSkeleton();
		// Draw articulation :
		glColor3f(1, 0, 0),
			gluSphere(gluNewQuadric(), 0.25, 10, 10);
		// Draw bone and children :
		glColor3f(0, 0, 1);
		for (unsigned int ichild = 0; ichild < _children.size(); ichild++) {
			drawBone(_children[ichild]);
			_children[ichild]->draw();
		}
	}
	glPopMatrix();
}

void Skeleton::animate(int iframe)
{
	// Update dofs :
	_curTx = 0; _curTy = 0; _curTz = 0;
	_curRx = 0; _curRy = 0; _curRz = 0;
	for (unsigned int idof = 0; idof < _dofs.size(); idof++) {
		if (!_dofs[idof].name.compare("Xposition")) _curTx = _dofs[idof]._values[iframe];
		if (!_dofs[idof].name.compare("Yposition")) _curTy = _dofs[idof]._values[iframe];
		if (!_dofs[idof].name.compare("Zposition")) _curTz = _dofs[idof]._values[iframe];
		if (!_dofs[idof].name.compare("Zrotation")) _curRz = _dofs[idof]._values[iframe];
		if (!_dofs[idof].name.compare("Yrotation")) _curRy = _dofs[idof]._values[iframe];
		if (!_dofs[idof].name.compare("Xrotation")) _curRx = _dofs[idof]._values[iframe];
	}
	// Animate children :
	for (unsigned int ichild = 0; ichild < _children.size(); ichild++) {
		_children[ichild]->animate(iframe);
	}
}

void Skeleton::eulerToMatrix(double rx, double ry, double rz, int rorder, glm::mat3 *R)
{
	glm::mat3 mx, my, mz;
	mx = glm::mat3(glm::vec3(1, 0, 0), glm::vec3(0, cos(rx), -sin(rx)), glm::vec3(0, sin(rx), cos(rx)));
	my = glm::mat3(glm::vec3(cos(ry), 0, sin(ry)), glm::vec3(0, 1, 0), glm::vec3(-sin(ry), 0, cos(ry)));
	mz = glm::mat3(glm::vec3(cos(rz), -sin(rz), 0), glm::vec3(sin(rz), cos(rz), 0), glm::vec3(0, 0, 1));
	switch (rorder) {
	case 0:
		*R = mx*my*mz;
		break;
	case 1:
		*R = my*mz*mx;
		break;
	case 2:
		*R = mz*mx*my;
		break;
	case 3:
		*R = mx*mz*my;
		break;
	case 4:
		*R = my*mx*mz;
		break;
	case 5:
		*R = mz*my*mx;
		break;
	}

}
void Skeleton::matrixToQuaternion(glm::mat3 R, qglviewer::Quaternion *q)
{
	double q3 = sqrt(1.0 + R[0][0] + R[1][1] + R[2][2]) / 2.0;
	double q0 = (R[2][1] - R[1][2]) / (4 * q3);
	double q1 = (R[0][2] - R[2][0]) / (4 * q3);
	double q2 = (R[1][0] - R[0][1]) / (4 * q3);
	q->setValue(q0, q1, q2, q3);
}

void Skeleton::quaternionToAxisAngle(qglviewer::Quaternion q, qglviewer::Vec *vaa)
{
	double angle = 2.0*acos(q[3]);
	qglviewer::Vec axis = qglviewer::Vec(q[0], q[1], q[2]);
	const double sinus = sqrt(q[1] * q[1] + q[2] * q[2] + q[0] * q[0]);
	if (sinus > 1E-8)
		axis /= sinus;

	if (angle > M_PI)
	{
		angle = 2.0*M_PI - angle;
		axis = -axis;
	}
	*vaa = axis*angle;
}

void Skeleton::eulerToAxisAngle(double rx, double ry, double rz, int rorder, qglviewer::Vec *vaa)
{
	// Euler -> matrix :
	glm::mat3 R;
	eulerToMatrix(M_PI*rx / 180.0, M_PI*ry / 180.0, M_PI*rz / 180.0, rorder, &R);
	// matrix -> quaternion :
	qglviewer::Quaternion q;
	matrixToQuaternion(R, &q);
	// quaternion -> axis/angle :
	quaternionToAxisAngle(q, vaa);
}


void Skeleton::nbDofs() {
	if (_dofs.empty()) return;

	double tol = 1e-4;

	int nbDofsR = 0;

	int isImplemented = 1;

	double rx, ry, rz;
	qglviewer::Vec vaa_prec, vaa;
	double angle, angle_prec;

    //Fichiers pour conserver les valeurs des axes et des angles
	ofstream file_axis(_name + "_axis.txt", ios::out | ios::trunc);
	ofstream file_angle(_name + "_angle.txt", ios::out | ios::trunc);

	for (unsigned int j = 0; j < _dofs[0]._values.size(); j++) {
		switch (_rorder) {
		case roXYZ:
			rx = _dofs[_dofs.size() - 3]._values[j];
			ry = _dofs[_dofs.size() - 2]._values[j];
			rz = _dofs[_dofs.size() - 1]._values[j];
			break;
		case roYZX:
			rx = _dofs[_dofs.size() - 2]._values[j];
			ry = _dofs[_dofs.size() - 1]._values[j];
			rz = _dofs[_dofs.size() - 3]._values[j];
			break;
		case roZXY:
			rx = _dofs[_dofs.size() - 1]._values[j];
			ry = _dofs[_dofs.size() - 3]._values[j];
			rz = _dofs[_dofs.size() - 2]._values[j];
			break;
		case roXZY:
			rx = _dofs[_dofs.size() - 3]._values[j];
			ry = _dofs[_dofs.size() - 1]._values[j];
			rz = _dofs[_dofs.size() - 2]._values[j];
			break;
		case roYXZ:
			rx = _dofs[_dofs.size() - 2]._values[j];
			ry = _dofs[_dofs.size() - 3]._values[j];
			rz = _dofs[_dofs.size() - 1]._values[j];
			break;
		case roZYX:
			rx = _dofs[_dofs.size() - 1]._values[j];
			ry = _dofs[_dofs.size() - 2]._values[j];
			rz = _dofs[_dofs.size() - 3]._values[j];
			break;
		}
        if (j == 0) {
		eulerToAxisAngle(rx, ry, rz, _rorder, &vaa_prec);
        angle_prec = vaa_prec.norm();
        vaa_prec.normalize();
        } else {
        //conversion euler->axis-angle
        eulerToAxisAngle(rx, ry, rz, _rorder, &vaa);
        angle = vaa.norm();
        vaa.normalize();
        
        //ecriture des valeurs dans les fichiers
        file_axis << j << " " << vaa.x << " " << vaa.y << " " << vaa.z << endl;
        file_angle << j << " " << angle << endl;
            
        //comparaison avec les valeurs precedentes
        double val = (vaa_prec - vaa).norm();
        double valOpp = (vaa_prec + vaa).norm(); //si orientation opposée
        if (((val > tol) && (valOpp > 2 - tol)) || ((valOpp > tol) && (val > 2 - tol))) {
            nbDofsR = 3;
            break;
        }
        else if ((angle_prec - angle) > tol) {
            nbDofsR = 1;
        }
            
        //Mises a jour
        vaa_prec = vaa;
        angle_prec = angle;
        }
	}
    
    //fermeture des fichiers
	file_axis.close();
	file_angle.close();
    
	if (!isImplemented) return;
	cout << _name << " : " << nbDofsR << " degree(s) of freedom in rotation\n";

	// Propagate to children :
	for (unsigned int ichild = 0; ichild < _children.size(); ichild++) {
		_children[ichild]->nbDofs();
	}

}
