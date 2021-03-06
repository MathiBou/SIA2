#include "skinning.h"

using namespace std;

void Skinning::init() {
	if (_skin==NULL) return;
	if (_skel==NULL) return;

	// Compute number of joints :
	getJoints(_skel);
	_nbJoints = _joints.size();

	// Get mesh info :
	_nbVtx = _skin->_points.size();
	_weights.resize(_nbVtx);
	_pointsInit.resize(_nbVtx);
	for (int iv = 0 ; iv < _nbVtx ; iv++) {
		_weights[iv].resize(_nbJoints, 0);
		_pointsInit[iv] = _skin->_points[iv];
		_pointsInit[iv][3] = 1.0;
	}

	// Get transfo joint info :
	_transfoInit.resize(_nbJoints);
	_transfoInitInv.resize(_nbJoints);
	_transfoCurr.resize(_nbJoints);
	int idx = 0;
	glPushMatrix();
	glLoadIdentity();
	computeTransfo(_skel, &idx);
	glPopMatrix();
	for (unsigned int i = 0 ; i < _transfoCurr.size() ; i++) {
		_transfoInit[i] = _transfoCurr[i];
		_transfoInitInv[i] = glm::inverse(_transfoInit[i]);
	}

	// Get bones pose info :
	idx = 0;
	_posBonesInit.resize(_nbJoints);
	getBonesPos(_skel, &idx);

	// Compute weights :
	if (_meth)
		computeWeights();
	else 
		loadWeights("data/skinning.txt");

	// Test skinning :
	animate();

}

void Skinning::recomputeWeights() {
	if (_skin==NULL) return;
	if (_skel==NULL) return;

	// Compute weights :
	if (!_meth) {
		if (_smoothSkinning) {
			computeWeightsSmooth();
		}
		else {
			computeWeights();
		}
		_smoothSkinning = !_smoothSkinning;
	} else {
		cout << "loading weights\n";
		loadWeights("data/skinning.txt");
	}

	// Test skinning :
	animate();
}

void Skinning::getJoints(Skeleton *skel) {
	_joints.push_back(skel);
	for (unsigned int ichild = 0 ; ichild < skel->_children.size() ; ichild++) {
		getJoints(skel->_children[ichild]);
	}
}
void Skinning::getBonesPos(Skeleton *skel, int *idx) {
	int i0 = (*idx);
	qglviewer::Vec pos(_transfoInit[i0][3][0], _transfoInit[i0][3][1], _transfoInit[i0][3][2]);
	for (unsigned int ichild = 0 ; ichild < skel->_children.size() ; ichild++) {
		(*idx)++;
		pos+=qglviewer::Vec(_transfoInit[(*idx)][3][0], _transfoInit[(*idx)][3][1], _transfoInit[(*idx)][3][2]);
		getBonesPos(skel->_children[ichild], idx);
	}
	pos/=(float)(skel->_children.size()+1);
	_posBonesInit[i0] = glm::vec4(pos.x, pos.y, pos.z, 1.0);
}

void Skinning::computeTransfo(Skeleton *skel, int *idx) {
	int i0 = (*idx);
	glPushMatrix();
	{
		glTranslatef(skel->_offX, skel->_offY, skel->_offZ);
		glTranslatef(skel->_curTx, skel->_curTy, skel->_curTz);
		skel->rotateSkeleton();

		float ptr[16];
		glGetFloatv(GL_MODELVIEW_MATRIX, ptr);
		int i = 0;
		for (int j = 0 ; j < 4 ; j++) {
			for (int k = 0 ; k < 4 ; k++) {
				_transfoCurr[(*idx)][k][j] = ptr[i];
				i++;
			}
		}
		for (unsigned int ichild = 0 ; ichild < skel->_children.size() ; ichild++) {
			(*idx)++;
			computeTransfo(skel->_children[ichild], idx);
		}
	}
	glPopMatrix();
	_transfoCurr[i0] = glm::transpose(_transfoCurr[i0]);
}

double Skinning::cylindricalDistance(glm::vec3 vert, int pere) {
	int fils;
	glm::vec3 jointA,jointB;
	double dist;
	
    //Position du joint pere
    jointA = glm::vec3(_transfoInit[pere][3][0], _transfoInit[pere][3][1], _transfoInit[pere][3][2]);
    
    //Recherche du joint fils
	for (unsigned int ichild = 0; ichild < _joints.at(pere)->_children.size(); ichild++) {
		Skeleton *child = _joints.at(pere)->_children[ichild];
		for (int m = 0; m < _nbJoints; m++) {
			if (_joints.at(m) == child) {
				fils = m;
			}
		}
		if (_joints.at(pere)->_children.size() != 0) {
            //Position du joint fils
			jointB = glm::vec3(_transfoInit[fils][3][0], _transfoInit[fils][3][1], _transfoInit[fils][3][2]);
            
            //Calcul de la distance cylindrique
			glm::vec3 u1 = vert - jointA;
			glm::vec3 u2 = jointB - jointA;
			float norm_u2 = sqrt(u2.x*u2.x + u2.y*u2.y + u2.z*u2.z);
			glm::vec3 cross = glm::cross(u1, u2);
			float norm_cross = sqrt(cross.x*cross.x + cross.y*cross.y + cross.z*cross.z);
            
			if (norm_u2 == 0) {
                //Si pere et fils sont a la meme position
				dist = glm::distance(jointA, vert);
			}
			else {
				float p = glm::dot(u1, u2) / pow(norm_u2, 2);
				if ((p >= 0 && p <= 1)) {
					dist = norm_cross / (norm_u2);
				}
				else if (p < 0) {
					dist = glm::distance(jointA, vert);
				}
				else {
					dist = glm::distance(jointB, vert);
				}

			}
		}
	}
	return dist;
}

void Skinning::computeWeightsSmooth() {
	if (_skin == NULL) return;
	if (_skel == NULL) return;

	cout << "compute smooth weights" << endl;
    
    if (_cylindricalDistance) {
	double weight;
	for (int i = 0; i < _nbVtx; i++) {
		glm::vec4 vertex = _pointsInit[i];
        glm::vec3 vert = glm::vec3(vertex);
		double sum = 0;
		for (int j = 0; j < _nbJoints; j++) {
            //Calcul du poids
			weight = exp(-cylindricalDistance(vert, j));
			_weights[i][j] = weight;
			sum += weight;
		}
        //normalisation des poids
		for (int j = 0; j < _nbJoints; j++) {
			_weights[i][j] /= sum;
		}
	}
    } else {
    double weight;
    for (int i = 0; i < _nbVtx; i++) {
        glm::vec4 vertex = _pointsInit[i];
        glm::vec3 vert = glm::vec3(vertex);
        double sum = 0;
        for (int j = 0; j < _nbJoints; j++) {
            //calcul du poids
            glm::vec4 joint = _posBonesInit[j];
            weight = exp(-glm::distance(vertex, joint));
            _weights[i][j] = weight;
            sum += weight;
        }
        //normalisation des poids
        for (int j = 0; j < _nbJoints; j++) {
            _weights[i][j] /= sum;
        }
    }
    }
}


void Skinning::computeWeights() {
	if (_skin==NULL) return;
	if (_skel==NULL) return;

    cout << "compute rigid weights" << endl;

	/*
	pour chaque vertex v
		pour chaque joint j
			on calcule la distance de v au joint
			on conserve l'index du joint le plus proche de v
		le bone le plus proche sera celui qui influera le vertex

	Tous les poids w[i][j] sont mis à zero, sauf celui correspondant au joint le plus proche qui vaut alors 1
	
	*/

	 for (int i = 0; i < _nbVtx; i++) {
		glm::vec4 vertex = _pointsInit[i];
		double minDist = numeric_limits<double>::infinity();
		int minJoint;
		for (int j = 0; j < _nbJoints; j++) {
			
			_weights[i][j] = 0;
			glm::vec4 joint = _posBonesInit[j];
			glm::vec3 vert = glm::vec3(vertex);
			
			if (glm::distance(vertex, joint)< minDist) {
				minDist = glm::distance(vertex, joint);
				minJoint = j;
			}
		}
		_weights[i][minJoint] = 1;
	} 
}

void Skinning::loadWeights(std::string filename) {
	std::vector<float> bone_indexA;
	std::vector<float> bone_weightA;
	FILE *file; fopen_s(&file, filename.data(), "r");
	if (!file) return;
	char * pch, *next_token;
	const int line_size = 600;
	char line[line_size];
	int iV = 0;
	while (!feof(file)) {
		// for each line i.e. for each vertex :
		if (fgets(line, line_size, file)) {
			int iJt = 0;
			float x;
			pch = strtok_s(line," ", &next_token);
			while (pch != NULL) {
				// for each number i.e. for each joint :
				if (pch[0]=='\n'){
				} else {
					x = (float)atof(pch);
					_weights[iV][iJt] = x;
				}
				pch = strtok_s(NULL, " ", &next_token);
				iJt++;
			}
			iV++;
		}		
	}
	fclose(file);
}

void Skinning::paintWeights(std::string jointName) {
	if (_skin == NULL) return;
	if (_skel == NULL) return;

	//On récupére l'indice du joint "jointName" 
	int idJoint;
	for (int j = 0; j < _nbJoints; j++) {
		if (!_joints.at(j)->_name.compare(jointName)) {
			idJoint = j;
		}
	}
	//On initialise le vectur de couleurs
	_skin->_colors = std::vector<glm::vec4>(_nbVtx);
	//On met à jour _skin->_colors
	for (int i = 0; i < _nbVtx; i++) {
		_skin->_colors[i] = (glm::vec4(_weights[i][idJoint], 0.0, 0.0, 0.0));
	}
}

void Skinning::animate() {
	if (_skin==NULL) return;
	if (_skel==NULL) return;

	// Animate bones :
	int idx = 0;
	glPushMatrix();
	glLoadIdentity();
	computeTransfo(_skel, &idx);
	glPopMatrix();

	// Animate skin :
#if _SKINNING_GPU
#else
	applySkinning();
#endif
}

void Skinning::applySkinning() {

	for (int i = 0; i < _nbVtx; i++) {
		_skin->_points[i] = glm::vec4(0.0, 0.0, 0.0, 0.0);
			for (int j = 0; j < _nbJoints; j++) {
				_skin->_points[i] += _weights[i][j] * _transfoCurr[j] * _transfoInitInv[j] * _pointsInit[i];
            }
		}
	
}
