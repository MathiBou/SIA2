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

Skeleton* Skeleton::setPoseInterpolation(std::string file1, int lastframe1, std::string file2, int firstframe2, float interpolationValue) {

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
		*R = mx * my * mz;
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
	/*float trace;
	trace = R[0][0] + R[1][1] + R[2][2];
	double q0, q1, q2, q3;
	if (trace >= 0)
	{
	// Direct computation
	const double s = sqrt(trace + 1.0) * 2.0;
	q0 = (R[2][1] - R[1][2]) / s;
	q1 = (R[0][2] - R[2][0]) / s;
	q2 = (R[1][0] - R[0][1]) / s;
	q3 = 0.25 * s;
	}
	else
	{
	// Computation depends on major diagonal term
	if ((R[0][0] > R[1][1])&(R[0][0] > R[2][2]))
	{
	const double s = sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2.0;
	q0 = 0.25 * s;
	q1 = (R[0][1] + R[1][0]) / s;
	q2 = (R[0][2] + R[2][0]) / s;
	q3 = (R[1][2] - R[2][1]) / s;
	}
	else
	if (R[1][1] > R[2][2])
	{
	const double s = sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2.0;
	q0 = (R[0][1] + R[1][0]) / s;
	q1 = 0.25 * s;
	q2 = (R[1][2] + R[2][1]) / s;
	q3 = (R[0][2] - R[2][0]) / s;
	}
	else
	{
	const double s = sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2.0;
	q0 = (R[0][2] + R[2][0]) / s;
	q1 = (R[1][2] + R[2][1]) / s;
	q2 = 0.25 * s;
	q3 = (R[0][1] - R[1][0]) / s;
	}
	}
	q->setValue(q0, q1, q2, q3);
	(*q).normalize();
	*/
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
	//qglviewer::Vec axis2 = q.axis();
	//qreal angle2 = q.angle();


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

	// TO COMPLETE :
	int isImplemented = 1;

	double rx, ry, rz;
	qglviewer::Vec vaa_prec, vaa;
	double angle, angle_prec;

	switch (_rorder) {
	case roXYZ:
		rx = _dofs[_dofs.size() - 3]._values[0];
		ry = _dofs[_dofs.size() - 2]._values[0];
		rz = _dofs[_dofs.size() - 1]._values[0];
		break;
	case roYZX:
		rx = _dofs[_dofs.size() - 2]._values[0];
		ry = _dofs[_dofs.size() - 1]._values[0];
		rz = _dofs[_dofs.size() - 3]._values[0];
		break;
	case roZXY:
		rx = _dofs[_dofs.size() - 1]._values[0];
		ry = _dofs[_dofs.size() - 3]._values[0];
		rz = _dofs[_dofs.size() - 2]._values[0];
		break;
	case roXZY:
		rx = _dofs[_dofs.size() - 3]._values[0];
		ry = _dofs[_dofs.size() - 1]._values[0];
		rz = _dofs[_dofs.size() - 2]._values[0];
		break;
	case roYXZ:
		rx = _dofs[_dofs.size() - 2]._values[0];
		ry = _dofs[_dofs.size() - 3]._values[0];
		rz = _dofs[_dofs.size() - 1]._values[0];
		break;
	case roZYX:
		rx = _dofs[_dofs.size() - 1]._values[0];
		ry = _dofs[_dofs.size() - 2]._values[0];
		rz = _dofs[_dofs.size() - 3]._values[0];
		break;
	}
	eulerToAxisAngle(rx, ry, rz, _rorder, &vaa_prec);
	angle_prec = vaa_prec.norm();
	vaa_prec.normalize();

	ofstream file_axis(_name + "_axis.txt", ios::out | ios::trunc);
	file_axis << 0 << " " << vaa_prec.x << " " << vaa_prec.y << " " << vaa_prec.z << endl;
	ofstream file_angle(_name + "_angle.txt", ios::out | ios::trunc);
	file_angle << 0 << " " << angle_prec << endl;

	for (unsigned int j = 1; j < _dofs[0]._values.size(); j++) {
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
		eulerToAxisAngle(rx, ry, rz, _rorder, &vaa);

		angle = vaa.norm();
		vaa.normalize();
		file_axis << j << " " << vaa.x << " " << vaa.y << " " << vaa.z << endl;
		file_angle << j << " " << angle << endl;

		double val = (vaa_prec - vaa).norm();
		double valOpp = (vaa_prec + vaa).norm(); //si orientation opposée
		if (((val > tol) && (valOpp > 2 - tol)) || ((valOpp > tol) && (val > 2 - tol))) {
			//if (val > tol) {
			nbDofsR = 2;
			//break;
		}
		else if ((angle_prec - angle) > tol) {
			nbDofsR = 1;
		}
		// Update vaaPrec
		vaa_prec = vaa;
		angle_prec = angle;
	}

	file_axis.close();
	file_angle.close();
	if (!isImplemented) return;
	cout << _name << " : " << nbDofsR << " degree(s) of freedom in rotation\n";

	// Propagate to children :
	for (unsigned int ichild = 0; ichild < _children.size(); ichild++) {
		_children[ichild]->nbDofs();
	}

}
