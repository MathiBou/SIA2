#include <sstream>

#include "skeletonIO.h"

using namespace std;

Skeleton* readHierarchy(std::ifstream &inputfile) {
	string word;
	// Find the ROOT key word
	inputfile >> word;
	size_t foundKeyWord;
	foundKeyWord = word.find("ROOT");
	while (foundKeyWord == string::npos) {
		inputfile >> word;
		foundKeyWord = word.find("ROOT");
	}
	// Get the ROOT name
	inputfile >> word;
	string rootName = word;
	Skeleton* root;

	// Get all the attributes for ROOT
	float offsetX, offsetY, offsetZ;
	inputfile >> word;
	if (word.find("{") != string::npos) {
		// Get the OFFSET
		inputfile >> word;
		foundKeyWord = word.find("OFFSET");
		if (foundKeyWord == string::npos) {
			// problem
		}
		// OFFSET X
		inputfile >> word;
		istringstream(word) >> offsetX;
		// OFFSET Y
		inputfile >> word;
		istringstream(word) >> offsetY;
		// OFFSET Z
		inputfile >> word;
		istringstream(word) >> offsetZ;

		// Get the CHANNELS
		inputfile >> word;
		foundKeyWord = word.find("CHANNELS");
		if (foundKeyWord == string::npos) {
			// problem
		}
		// Number of channels
		int counter;
		inputfile >> word;
		istringstream(word) >> counter;
		// Get all the channels
		vector<AnimCurve> dofs;
		for (int i = 0; i < counter; i++) {
			inputfile >> word;
			AnimCurve channel;
			channel.name = word;
			dofs.push_back(channel);
		}

		// We can create the root
		root = Skeleton::create(rootName, offsetX, offsetY, offsetZ, NULL);
		root->_dofs = dofs;
		defineRotateOrder(root);

		// Recursively get all the joints
		while (word.find("MOTION") == string::npos) {
			inputfile >> word;
			foundKeyWord = word.find("JOINT");
			if (foundKeyWord != string::npos) {
				readJoint(inputfile, root);
			}
		}

	}

	// The keyword MOTION was read
	readMotion(inputfile, root);

	return root;
}

Skeleton* interpolate(std::ifstream &inputfile1, std::ifstream &inputfile2) {

	cout << "interpolate" << endl;

	string word;
	// Find the ROOT key word
	inputfile1 >> word;
	inputfile2 >> word;

	size_t foundKeyWord;
	foundKeyWord = word.find("ROOT");
	while (foundKeyWord == string::npos) {
		inputfile1 >> word;
		inputfile2 >> word;

		foundKeyWord = word.find("ROOT");
	}
	// Get the ROOT name
	inputfile1 >> word;
	inputfile2 >> word;

	string rootName = word;
	Skeleton* root;

	// Get all the attributes for ROOT
	float offsetX, offsetY, offsetZ;
	inputfile1 >> word;
	inputfile2 >> word;

	if (word.find("{") != string::npos) {
		// Get the OFFSET
		inputfile1 >> word;
		inputfile2 >> word;

		foundKeyWord = word.find("OFFSET");
		if (foundKeyWord == string::npos) {
			// problem
		}
		// OFFSET X
		inputfile1 >> word;
		inputfile2 >> word;

		istringstream(word) >> offsetX;
		// OFFSET Y
		inputfile1 >> word;
		inputfile2 >> word;

		istringstream(word) >> offsetY;
		// OFFSET Z
		inputfile1 >> word;
		inputfile2 >> word;

		istringstream(word) >> offsetZ;

		// Get the CHANNELS
		inputfile1 >> word;
		inputfile2 >> word;

		foundKeyWord = word.find("CHANNELS");
		if (foundKeyWord == string::npos) {
			// problem
		}
		// Number of channels
		int counter;
		inputfile1 >> word;
		inputfile2 >> word;

		istringstream(word) >> counter;
		// Get all the channels
		vector<AnimCurve> dofs;
		for (int i = 0; i < counter; i++) {
			inputfile1 >> word;
			inputfile2 >> word;

			AnimCurve channel;
			channel.name = word;
			dofs.push_back(channel);
		}

		// We can create the root
		root = Skeleton::create(rootName, offsetX, offsetY, offsetZ, NULL);
		root->_dofs = dofs;
		defineRotateOrder(root);

		// Recursively get all the joints
		while (word.find("MOTION") == string::npos) {
			inputfile1 >> word;
			inputfile2 >> word;

			foundKeyWord = word.find("JOINT");
			if (foundKeyWord != string::npos) {
				interpolateJoint(inputfile1, inputfile2, root);
			}
		}

	}

	interpolateMotion(inputfile1, inputfile2, root);

	return root;
}

Skeleton* transition(std::ifstream &inputfile1, std::ifstream &inputfile2) {

	cout << "transition" << endl;

	string word;
	// Find the ROOT key word
	inputfile1 >> word;
	inputfile2 >> word;

	size_t foundKeyWord;
	foundKeyWord = word.find("ROOT");
	while (foundKeyWord == string::npos) {
		inputfile1 >> word;
		inputfile2 >> word;

		foundKeyWord = word.find("ROOT");
	}
	// Get the ROOT name
	inputfile1 >> word;
	inputfile2 >> word;

	string rootName = word;
	Skeleton* root;

	// Get all the attributes for ROOT
	float offsetX, offsetY, offsetZ;
	inputfile1 >> word;
	inputfile2 >> word;

	if (word.find("{") != string::npos) {
		// Get the OFFSET
		inputfile1 >> word;
		inputfile2 >> word;

		foundKeyWord = word.find("OFFSET");
		if (foundKeyWord == string::npos) {
			// problem
		}
		// OFFSET X
		inputfile1 >> word;
		inputfile2 >> word;

		istringstream(word) >> offsetX;
		// OFFSET Y
		inputfile1 >> word;
		inputfile2 >> word;

		istringstream(word) >> offsetY;
		// OFFSET Z
		inputfile1 >> word;
		inputfile2 >> word;

		istringstream(word) >> offsetZ;

		// Get the CHANNELS
		inputfile1 >> word;
		inputfile2 >> word;

		foundKeyWord = word.find("CHANNELS");
		if (foundKeyWord == string::npos) {
			// problem
		}
		// Number of channels
		int counter;
		inputfile1 >> word;
		inputfile2 >> word;

		istringstream(word) >> counter;
		// Get all the channels
		vector<AnimCurve> dofs;
		for (int i = 0; i < counter; i++) {
			inputfile1 >> word;
			inputfile2 >> word;

			AnimCurve channel;
			channel.name = word;
			dofs.push_back(channel);
		}

		// We can create the root
		root = Skeleton::create(rootName, offsetX, offsetY, offsetZ, NULL);
		root->_dofs = dofs;
		defineRotateOrder(root);

		// Recursively get all the joints
		while (word.find("MOTION") == string::npos) {
			inputfile1 >> word;
			inputfile2 >> word;

			foundKeyWord = word.find("JOINT");
			if (foundKeyWord != string::npos) {
				interpolateJoint(inputfile1, inputfile2, root);
			}
		}

	}
	int nbTransitionFrames = 20;
	double deltaX, deltaY, deltaZ = 0; 

	std::vector<double> offset = transitionMotion(inputfile1, root, 0, 0, 0);
	cout << "offset" << offset[0] << " " << offset[1] << " " << offset[2] << endl;
	
	int nbFrames = root->_dofs.at(0)._values.size();
	
	skipTransitionFrames(nbTransitionFrames, root);
	
	deltaX = abs(root->_dofs.at(0)._values.at(0) - root->_dofs.at(0)._values.at(nbTransitionFrames)); 
	deltaY = abs(root->_dofs.at(1)._values.at(0) - root->_dofs.at(1)._values.at(nbTransitionFrames));
	deltaZ = abs(root->_dofs.at(2)._values.at(0) - root->_dofs.at(2)._values.at(nbTransitionFrames));
	cout << "delta" << deltaX << " " << deltaY << " " << deltaZ << endl;

	std::vector<double> osef = transitionMotion(inputfile2, root, offset[0] + deltaX, offset[1] + deltaY, offset[2] + deltaZ);
	updateTransitionFrames(nbFrames, nbTransitionFrames, root);
	cout << "passed both trMotion" << endl;
	return root;
}

void updateTransitionFramesQuaternion(int nbFrames, int nbTransitionFrames, Skeleton* root) {
	
	cout << "update transition frames quaternion" << endl;

	bool isRoot = (root->_name == "hip");
	glm::mat3 RFirst, RLast;
	qglviewer::Quaternion qFirst, qLast;
	float rx, ry, rz, px, py, pz;
	if (isRoot) {

		cout << "IS ROOT" << endl;
		//get last position of walk
		px = root->_dofs.at(0)._values.at(nbFrames - 1);
		py = root->_dofs.at(1)._values.at(nbFrames - 1);
		pz = root->_dofs.at(2)._values.at(nbFrames - 1);

		//get first quaternion
		rx = root->_dofs.at(3)._values.at(nbFrames - 1);
		ry = root->_dofs.at(4)._values.at(nbFrames - 1);
		rz = root->_dofs.at(5)._values.at(nbFrames - 1);



		Skeleton::eulerToMatrix(M_PI*rx / 180.0, M_PI*ry / 180.0, M_PI*rz / 180.0, 0, &RFirst);
		cout << "after eulerToMatrix" << endl; 
		Skeleton::matrixToQuaternion(RFirst, &qFirst);
		cout << "after matrix to Quaternion" << endl;

		//get last quaternion
		rx = root->_dofs.at(3)._values.at(nbFrames + nbTransitionFrames);
		ry = root->_dofs.at(4)._values.at(nbFrames + nbTransitionFrames);
		rz = root->_dofs.at(5)._values.at(nbFrames + nbTransitionFrames);
		Skeleton::eulerToMatrix(rx, ry, rz, 0, &RLast);
		cout << "after eulerToMatrix" << endl;

		Skeleton::matrixToQuaternion(RLast, &qLast);
		cout << "after matrix to Quaternion" << endl;
	}
	else {
		cout << "NOT ROOT" << endl;
		cout << root->_name << endl; 
		if (root->_name == "End") {
			cout << "END ??? " << endl;
			goto Skip; 
		}
		//get first quaternion
		rx = root->_dofs.at(0)._values.at(nbFrames - 1);
		ry = root->_dofs.at(1)._values.at(nbFrames - 1);
		rz = root->_dofs.at(2)._values.at(nbFrames - 1);
		cout << "got first euler angle" << endl;
		Skeleton::eulerToMatrix(rx, ry, rz, 0, &RFirst);
		cout << "passed euler to matrix" << endl;
		Skeleton::matrixToQuaternion(RFirst, &qFirst); 
		cout << "passed matrix to quaternion" << endl;

		//get last quaternion

		cout << "size" << root->_dofs.at(0)._values.size() << " " << nbFrames + nbTransitionFrames << endl;

		rx = root->_dofs.at(0)._values.at(nbFrames + nbTransitionFrames);
		ry = root->_dofs.at(1)._values.at(nbFrames + nbTransitionFrames);
		rz = root->_dofs.at(2)._values.at(nbFrames + nbTransitionFrames);
		cout << "got second euler angle" << endl;

		Skeleton::eulerToMatrix(rx, ry, rz, 0, &RLast);
		cout << "passed euler to matrix" << endl;
		Skeleton::matrixToQuaternion(RLast, &qLast);
		cout << "passed matrix to quaternion" << endl;
	}

	qglviewer::Quaternion q;
	float t = 1; 
	double roll, pitch, yaw; 
	for (int i = nbFrames; i < (nbFrames + nbTransitionFrames); i++) {
		q = qglviewer::Quaternion::slerp(qFirst, qLast, t); 
		t -= (1.0 / (float)nbTransitionFrames); 
		//Skeleton::toEulerianAngle(q, roll, pitch, yaw);
		if (isRoot) {

			root->_dofs.at(0)._values[i] = px;
			root->_dofs.at(1)._values[i] = py;
			root->_dofs.at(2)._values[i] = pz;

			root->_dofs.at(3)._values[i] = roll; 
			root->_dofs.at(4)._values[i] = pitch;
			root->_dofs.at(5)._values[i] = yaw;
		}
		else {
			root->_dofs.at(0)._values[i] = roll;
			root->_dofs.at(1)._values[i] = pitch;
			root->_dofs.at(2)._values[i] = yaw;
		}
	}
	Skip :
	cout << "begin recursive call" << endl;
	// Recursive call for all its children
	for (Skeleton* child : root->_children) {
		cout << "recursive  " << child->_name << endl;
		updateTransitionFramesQuaternion(nbFrames, nbTransitionFrames, child);
	}
}

void updateTransitionFrames(int nbFrames, int nbTransitionFrames, Skeleton* root) {
	cout << "update frames" << endl;
	for (int j = 0; j < root->_dofs.size(); j++) {
		double first = root->_dofs.at(j)._values.at(nbFrames - 1);
		double last = root->_dofs.at(j)._values.at(nbFrames + nbTransitionFrames);
		double weight = 1;
		for (int i = nbFrames; i < (nbFrames + nbTransitionFrames); i++) {
			root->_dofs.at(j)._values[i] = weight*first + (1 - weight)*last;
			weight -= (1.0 / (double)nbTransitionFrames);
		}
	}

	// Recursive call for all its children
	for (Skeleton* child : root->_children) {
		updateTransitionFrames(nbFrames, nbTransitionFrames, child);
	}
}

void skipTransitionFrames(int nbTransitionFrames, Skeleton* root){

	for (int i = 0; i < nbTransitionFrames; i++) {
		for (int j = 0; j < root->_dofs.size(); j++) {
			root->_dofs.at(j)._values.push_back(0);
		}
	}

	// Recursive call for all its children
	for (Skeleton* child : root->_children) {
		skipTransitionFrames(nbTransitionFrames, child);
	}
}

void readJoint(std::ifstream &inputfile, Skeleton* parent) {
	Skeleton* child;
	string word;
	inputfile >> word;
	size_t foundKeyWord;
	foundKeyWord = word.find("Site");
	if (foundKeyWord == string::npos) {
		// New joint
		string jointName = word;

		// Get all the attributes
		float offsetX, offsetY, offsetZ;
		inputfile >> word;
		if (word.find("{") != string::npos) {
			// Get the OFFSET
			inputfile >> word;
			foundKeyWord = word.find("OFFSET");
			if (foundKeyWord == string::npos) {
				// problem
			}
			// OFFSET X
			inputfile >> word;
			istringstream(word) >> offsetX;
			// OFFSET Y
			inputfile >> word;
			istringstream(word) >> offsetY;
			// OFFSET Z
			inputfile >> word;
			istringstream(word) >> offsetZ;

			// Get the CHANNELS
			inputfile >> word;
			foundKeyWord = word.find("CHANNELS");
			if (foundKeyWord == string::npos) {
				// problem
			}
			// Number of channels
			int counter;
			inputfile >> word;
			istringstream(word) >> counter;
			// Get all the channels
			vector<AnimCurve> dofs;
			for (int i = 0; i < counter; i++) {
				inputfile >> word;
				AnimCurve channel;
				channel.name = word;
				dofs.push_back(channel);
			}

			// We can create the new joint
			child = Skeleton::create(jointName, offsetX, offsetY, offsetZ, parent);
			child->_dofs = dofs;
			defineRotateOrder(child);

			// Recursive calls
			inputfile >> word;
			while (word.find("}") == string::npos) {
				readJoint(inputfile, child);
				inputfile >> word;
			}
		}
	}
	else {
		// Terminal case
		string endJoint = "End";
		// Pass "{"
		inputfile >> word;

		// Get the OFFSET
		float offsetX, offsetY, offsetZ;
		inputfile >> word;
		foundKeyWord = word.find("OFFSET");
		if (foundKeyWord == string::npos) {
			// problem
		}
		// OFFSET X
		inputfile >> word;
		istringstream(word) >> offsetX;
		// OFFSET Y
		inputfile >> word;
		istringstream(word) >> offsetY;
		// OFFSET Z
		inputfile >> word;
		istringstream(word) >> offsetZ;

		// We can create the last joint, which has no children
		child = Skeleton::create(endJoint, offsetX, offsetY, offsetZ, parent);

		// Pass the closing bracket
		inputfile >> word;

		// Terminal case, we return
		return;
	}

}

void interpolateJoint(std::ifstream &inputfile, std::ifstream &inputfile2, Skeleton* parent) {
	Skeleton* child;
	string word;
	inputfile >> word;
	inputfile2 >> word;

	size_t foundKeyWord;
	foundKeyWord = word.find("Site");
	if (foundKeyWord == string::npos) {
		// New joint
		string jointName = word;

		// Get all the attributes
		float offsetX, offsetY, offsetZ;
		inputfile >> word;
		inputfile2 >> word;

		if (word.find("{") != string::npos) {
			// Get the OFFSET
			inputfile >> word;
			inputfile2 >> word;

			foundKeyWord = word.find("OFFSET");
			if (foundKeyWord == string::npos) {
				// problem
			}
			// OFFSET X
			inputfile >> word;
			inputfile2 >> word;

			istringstream(word) >> offsetX;
			// OFFSET Y
			inputfile >> word;
			inputfile2 >> word;

			istringstream(word) >> offsetY;
			// OFFSET Z
			inputfile >> word;
			inputfile2 >> word;

			istringstream(word) >> offsetZ;

			// Get the CHANNELS
			inputfile >> word;
			inputfile2 >> word;

			foundKeyWord = word.find("CHANNELS");
			if (foundKeyWord == string::npos) {
				// problem
			}
			// Number of channels
			int counter;
			inputfile >> word;
			inputfile2 >> word;
			istringstream(word) >> counter;
			// Get all the channels
			vector<AnimCurve> dofs;
			for (int i = 0; i < counter; i++) {
				inputfile >> word;
				inputfile2 >> word;

				AnimCurve channel;
				channel.name = word;
				dofs.push_back(channel);
			}

			// We can create the new joint
			child = Skeleton::create(jointName, offsetX, offsetY, offsetZ, parent);
			child->_dofs = dofs;
			defineRotateOrder(child);

			// Recursive calls
			inputfile >> word;
			inputfile2 >> word;

			while (word.find("}") == string::npos) {
				interpolateJoint(inputfile, inputfile2, child);
				inputfile >> word;
				inputfile2 >> word;

			}
		}
	}
	else {
		// Terminal case
		string endJoint = "End";
		// Pass "{"
		inputfile >> word;
		inputfile2 >> word;


		// Get the OFFSET
		float offsetX, offsetY, offsetZ;
		inputfile >> word;
		inputfile2 >> word;

		foundKeyWord = word.find("OFFSET");
		if (foundKeyWord == string::npos) {
			// problem
		}
		// OFFSET X
		inputfile >> word;
		inputfile2 >> word;

		istringstream(word) >> offsetX;
		// OFFSET Y
		inputfile >> word;
		inputfile2 >> word;

		istringstream(word) >> offsetY;
		// OFFSET Z
		inputfile >> word;
		inputfile2 >> word;

		istringstream(word) >> offsetZ;

		// We can create the last joint, which has no children
		child = Skeleton::create(endJoint, offsetX, offsetY, offsetZ, parent);

		// Pass the closing bracket
		inputfile >> word;
		inputfile2 >> word;


		// Terminal case, we return
		return;
	}

}

void readMotion(std::ifstream &inputfile, Skeleton* root) {
	string word;
	size_t foundKeyWord;

	// Get the number of frames
	inputfile >> word;
	foundKeyWord = word.find("Frames:");
	if (foundKeyWord == string::npos) {
		// problem
	}
	inputfile >> word;
	int nbFrames;
	istringstream(word) >> nbFrames;

	// Get the sampling rate
	inputfile >> word; inputfile >> word;
	foundKeyWord = word.find("Time:");
	if (foundKeyWord == string::npos) {
		// problem
	}
	inputfile >> word;
	float sampleRate;
	istringstream(word) >> sampleRate;

	// Read all the frames
	for (int i = 0; i < nbFrames; i++) {
		// The reading begins at the beginning of each word
		//HERE
		readKeyFrame(inputfile, root, true);
	}
}

std::vector<double> transitionMotion(std::ifstream &inputfile, Skeleton* root, double X, double Y, double Z) {
	string word;
	size_t foundKeyWord;

	// Get the number of frames
	inputfile >> word;
	foundKeyWord = word.find("Frames:");
	if (foundKeyWord == string::npos) {
		// problem
	}
	inputfile >> word;

	int nbFrames;
	istringstream(word) >> nbFrames;

	// Get the sampling rate
	inputfile >> word; inputfile >> word;
	foundKeyWord = word.find("Time:");
	if (foundKeyWord == string::npos) {
		// problem
	}
	inputfile >> word;
	float sampleRate;
	istringstream(word) >> sampleRate;

	double lastX, lastY, lastZ = 0;
	// Read all the frames
	//for (int i = 0; i < nbFrames; i++) {
	for (int i = 0; i < nbFrames; i++) {

		// The reading begins at the beginning of each word
		//if first frame
		if (i == 0)
		{
			readKeyFrame(inputfile, root, true, X, Y, Z);
		}
		//other frames
		else {
			readKeyFrame(inputfile, root, false, X, Y, Z);
		}
		//last frame
		if (i == nbFrames - 1) {
			lastX = root->_dofs[0]._values[i];
			lastY = root->_dofs[1]._values[i];
			lastZ = root->_dofs[2]._values[i];
			cout << lastX << " " << lastY << " " << lastZ << " " << endl;
		}
	}
	std::vector<double> vec;
	vec.push_back(lastX); 	vec.push_back(lastY); 	vec.push_back(lastZ);
	return vec;
}

void interpolateMotion(std::ifstream &inputfile1, std::ifstream &inputfile2, Skeleton* root) {

	cout << "interpolation Motion" << endl;

	string word;
	size_t foundKeyWord;

	// Get the number of frames
	inputfile1 >> word;
	foundKeyWord = word.find("Frames:");
	if (foundKeyWord == string::npos) {
		// problem
	}
	inputfile1 >> word;
	cout << word << endl;

	int nbFrames1;
	istringstream(word) >> nbFrames1;
	cout << nbFrames1 << endl;


	inputfile2 >> word;
	foundKeyWord = word.find("Frames:");
	if (foundKeyWord == string::npos) {
		// problem
	}
	inputfile2 >> word;
	int nbFrames2;
	istringstream(word) >> nbFrames2;
	cout << nbFrames2 << endl;


	int nbFrames;
	if (nbFrames1 < nbFrames2)
		nbFrames = nbFrames1;
	else nbFrames = nbFrames2;
	cout << "passed nb frames" << endl;
	cout << nbFrames << endl;

	// Get the sampling rate
	inputfile1 >> word; inputfile1 >> word;
	foundKeyWord = word.find("Time:");
	if (foundKeyWord == string::npos) {
		// problem
	}
	inputfile1 >> word;
	float sampleRate1;
	istringstream(word) >> sampleRate1;

	// Get the sampling rate
	inputfile2 >> word; inputfile2 >> word;
	foundKeyWord = word.find("Time:");
	if (foundKeyWord == string::npos) {
		// problem
	}
	inputfile2 >> word;
	float sampleRate2;
	istringstream(word) >> sampleRate2;

	float sampleRate = sampleRate1;
	if (sampleRate1 != sampleRate2)
		sampleRate = (sampleRate1 + sampleRate2) / 2.0;

	cout << "passed sample rate" << endl;

	// Read all the frames
	for (int i = 0; i < nbFrames; i++) {
		cout << i << endl;

		// The reading begins at the beginning of each word
		interpolateKeyFrame(inputfile1, inputfile2, root);
	}
}

void readKeyFrame(std::ifstream &inputfile, Skeleton* skel, bool isFirstFrame, double Xinit, double Yinit, double Zinit) {
	static double firstValX = 0; 
	static double firstValY = 0;
	static double firstValZ = 0;
	
	static ofstream file("Zpos.txt", ios::out | ios::trunc);

	string word;
	// Read the values for all the joint's own dofs
	if (!skel->_dofs.empty()) {
		for (size_t i = 0; i < skel->_dofs.size(); i++) {
			inputfile >> word;
			double val;
			istringstream(word) >> val;

			if (i == 0) {
				if ((isFirstFrame) && Xinit !=0) { //first frame de la deuxieme animation
					firstValX = val;
				}

				if (Xinit != 0) {
					skel->_dofs[i]._values.push_back(Xinit + (val - firstValX));
					//cout << "X position  " << skel->_name << "  "<< Xinit + (val - firstValX) << endl ;
				}
				else {
					skel->_dofs[i]._values.push_back(val + Xinit);
					//cout << "X position" << skel->_name << val + Xinit << endl ;

				}
			}
			else if (i == 1) {
				if ((isFirstFrame) && Yinit != 0) { //first frame de la deuxieme animation
					firstValY = val;
				}
				if (Yinit != 0) {
					skel->_dofs[i]._values.push_back(Yinit + (val - firstValY));
				}
				else {
					skel->_dofs[i]._values.push_back(val + Yinit);
				}
			}
			else if (i == 2) {
				if ((isFirstFrame) && Zinit != 0) { //first frame de la deuxieme animation
					firstValZ = val;
					isFirstFrame = false;
					file << "first frame" << endl;
					file << "Z val first frame" << val << endl;

				}
				if (Zinit != 0) {
					file << "is root ? " << skel->_name << endl;
					file << "Zinit" << Zinit << endl; 
					file << "firstValZ" << firstValZ << endl;
					file << "val" << val << endl;
					file << "Zpos " << Zinit + (val - firstValZ) << endl;

					skel->_dofs[i]._values.push_back(Zinit + (val - firstValZ));		
				}
				else {
					skel->_dofs[i]._values.push_back(val + Zinit);
				}
			}
			else{
				skel->_dofs[i]._values.push_back(val);
			}
		}
	}
	else {
		// Terminal case : End Site
		return;
	}

	// Recursive call for all its children
	for (Skeleton* child : skel->_children) {
		readKeyFrame(inputfile, child, 0, 0, 0);
	}
}

void interpolateKeyFrame(std::ifstream &inputfile1, std::ifstream &inputfile2, Skeleton* skel) {

	cout << "interpolateFrame" << endl;

	string word1, word2;
	// Read the values for all the joint's own dofs
	if (!skel->_dofs.empty()) {
		for (size_t i = 0; i < skel->_dofs.size(); i++) {
			inputfile1 >> word1;
			double val1;
			istringstream(word1) >> val1;

			inputfile2 >> word2;
			double val2;
			istringstream(word2) >> val2;

			double val = (val1 + val2) / 2.0;

			skel->_dofs[i]._values.push_back(val);
		}
	}
	else {
		// Terminal case : End Site
		return;
	}

	// Recursive call for all its children
	for (Skeleton* child : skel->_children) {
		interpolateKeyFrame(inputfile1, inputfile2, child);
	}
}

void defineRotateOrder(Skeleton *skel) {
	vector<AnimCurve> dofs = skel->_dofs;
	size_t i = dofs.size() - 3;
	string nameRot = skel->_dofs[i].name;
	while (i < skel->_dofs.size()) {
		if (!nameRot.compare("Xrotation")) {
			i++;
			nameRot = skel->_dofs[i].name;
			if (!nameRot.compare("Yrotation")) {
				skel->_rorder = roXYZ;
				return;
			}
			if (!nameRot.compare("Zrotation")) {
				skel->_rorder = roXZY;
				return;
			}
		}
		else if (!nameRot.compare("Yrotation")) {
			i++;
			nameRot = skel->_dofs[i].name;
			if (!nameRot.compare("Xrotation")) {
				skel->_rorder = roYXZ;
				return;
			}
			if (!nameRot.compare("Zrotation")) {
				skel->_rorder = roYZX;
				return;
			}
		}
		else if (!nameRot.compare("Zrotation")) {
			i++;
			nameRot = skel->_dofs[i].name;
			if (!nameRot.compare("Xrotation")) {
				skel->_rorder = roZXY;
				return;
			}
			if (!nameRot.compare("Yrotation")) {
				skel->_rorder = roZYX;
				return;
			}
		}
		else {
			i++;
		}
	}
}
