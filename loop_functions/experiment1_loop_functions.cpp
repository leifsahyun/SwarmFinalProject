#include "experiment1_loop_functions.h"

/****************************************/
/****************************************/

static const Real MAX_R = 10.0;


CMPGAExperiment1LoopFunctions::CMPGAExperiment1LoopFunctions() :
   m_vecInitSetup(5),
   m_pcFootBot(NULL),
   m_pfControllerParams(new Real[GENOME_SIZE]),
   m_pcRNG(NULL),
   avg_speed(0.0),
   scatter(0.0),
   ang_momentum(CVector2(0.0,0.0)),
   grp_rotation(CVector2(0.0,0.0)),
   rad_variance(0.0),
   swarm_centroid(CVector2(0.0,0.0)),
   num_robots(0),
   time_step(0) {}


struct GetRobotData : public CBuzzLoopFunctions::COperation {

	/** The action happens here */
	virtual void operator()(const std::string str_robot_id,buzzvm_t t_vm) {

		/*Get the pose of the robots*/
		BuzzTableOpen(t_vm, "pose.position");
		buzzobj_t tPose = BuzzGet(t_vm, "pose.position");
		/* Make sure it's the type we expect (a table) */
		if(!buzzobj_istable(tPose)) {
			LOGERR << str_robot_id << ": variable 'pose.position' has wrong type " << buzztype_desc[tPose->o.type] << std::endl;
			return;
		}

		m_vecRobotsPose[t_vm->robot] = CVector2(tPose[0], tPose[1]);

		BuzzTableOpen(t_vm, "velocity");
		buzzobj_t tVelocity = BuzzGet(t_vm, "velocity");
		if(!buzzobj_istable(tVelocity)) {
			LOGERR << str_robot_id << ": variable 'velocity' has wrong type " << buzztype_desc[tVelocity->o.type] << std::endl;
			return;
      	}

      	m_vecRobotVelocity[t_vm->robot] = CVector2(tVelocity[0],tVelocity[1]);
	}

	/*map for pose*/
	std::map<int, CVector2> m_vecRobotsPose;
	std::map<int, CVector2> m_vecRobotVelocity;
};


struct SetRobotVelocity : public CBuzzLoopFunctions::COperation {

	/** Constructor */
	SetRobotVelocity(const std::vector<Real>& vec_velocity) : m_vecVelocity(vec_veclocity) {}

	/** The action happens here */
	virtual void operator()(const std::string str_robot_id,buzzvm_t t_vm) {
		/* Set the values of the table 'stimulus' in the Buzz VM */
		BuzzTableOpen(t_vm, "robot_velocity");
		for(int i = 0; i < m_vecVelocity.size(); ++i) {
			BuzzTablePut(t_vm, i, m_vecVelocity[i]);
		}
		BuzzTableClose(t_vm);
	}

	/** Calculated stimuli */
	const std::vector<std::pair<Real,Real> >& m_vecVelocity;
};

inline int CMPGAExperiment1LoopFunctions::GetNumRobots() {
	return m_mapBuzzVMs.size();
}

/****************************************/
/****************************************/

void CMPGAExperiment1LoopFunctions::Init(TConfigurationNode& t_node) {
	/*
	* Create the random number generator
	*/
	ofs.open ("test.txt", std::ofstream::out | std::ofstream::app);

	m_pcRNG = CRandom::CreateRNG("argos");

	/*
	* Create the foot-bot and get a reference to its controller
	*/
	m_pcFootBot = new CFootBotEntity(
		"fb",    // entity id
		"fnn"    // controller id as set in the XML
	);
	AddEntity(*m_pcFootBot);
	// m_pcController = &dynamic_cast<CFootBotNNController&>(m_pcFootBot->GetControllableEntity().GetController());

	/*
	* Create the initial setup for each trial
	* The robot is placed 4.5 meters away from the light
	* (which is in the origin) at angles
	* { PI/12, 2*PI/12, 3*PI/12, 4*PI/12, 5*PI/12 }
	* wrt to the world reference.
	* Also, the rotation of the robot is chosen at random
	* from a uniform distribution.
	*/
	CRadians cOrient;
	for(size_t i = 0; i < 5; ++i) {
		/* Set position */
		m_vecInitSetup[i].Position.FromSphericalCoords(
			4.5f,                                          // distance from origin
			CRadians::PI_OVER_TWO,                         // angle with Z axis
			static_cast<Real>(i+1) * CRadians::PI / 12.0f // rotation around Z
		);
		/* Set orientation */
		cOrient = m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE);
		m_vecInitSetup[i].Orientation.FromEulerAngles(
			cOrient,        // rotation around Z
			CRadians::ZERO, // rotation around Y
			CRadians::ZERO  // rotation around X
		);
	}

	/*
	* Process trial information, if any
	*/
	try {
		UInt32 unTrial;
		GetNodeAttribute(t_node, "trial", unTrial);
		SetTrial(unTrial);
		Reset();
	}
	catch(CARGoSException& ex) {}
}


void CMPGAExperiment1LoopFunctions::PostStep() {
	/* Get robot data */
	GetRobotData cGetRobotData;
	BuzzForeachVM(cGetRobotData);
	int num_robots = GetNumRobots();
	/*Get swarm centroid*/
	Real avgX = 0.0, avgY = 0.0;
	for(auto itr = cGetRobotData.m_vecRobotsPose.begin(); itr!=cGetRobotData.m_vecRobotsPose.end(); itr++) {
		avgX += itr->second.GetX();
		avgY += itr->second.GetY();
	}
	swarm_centroid.SetX(avgX/num_robots);
	swarm_centroid.SetY(avgY/num_robots);

	/*Get the average velocity of the swarm*/
	Real avg_speedx = 0.0, avg_speedy = 0.0;
	for(auto itr = cGetRobotData.m_vecRobotVelocity.begin(); itr!=cGetRobotData.m_vecRobotVelocity.end(); itr++) {
		avg_speed  += std::sqrt(itr->second.GetX()*itr->second.GetX() + itr->second.GetY()*itr->second.GetY());
	}
	avg_speed = avg_speed/num_robots;

	/*Get the scatter for the swarm*/
	scatter = 0.0;
	for(auto itr = cGetRobotData.m_vecRobotsPose.begin(); itr!=cGetRobotData.m_vecRobotsPose.end(); itr++) {
		scatter += std::pow(itr->second.GetX() -  swarm_centroid.GetX(), 2) + std::pow(itr->second.GetY() -  swarm_centroid.GetY(), 2);
	}
	scatter /= (MAX_R*MAX_R*num_robots);

	/*Get the radial variance for the swarm*/
	rad_variance = 0.0;
	Real variance = 0.0;
	for(auto itr = cGetRobotData.m_vecRobotsPose.begin(); itr!=cGetRobotData.m_vecRobotsPose.end(); itr++) {
		variance += std::sqrt(std::pow(itr->second.GetX() -  swarm_centroid.GetX(), 2) + std::pow(itr->second.GetY() -  swarm_centroid.GetY(), 2));
	}
	variance /= (std::pow(MAX_R*num_robots),2);
	rad_variance = scatter - variance;

	/*Get the angular momentum*/
	ang_momentum.Set(0.0,0.0);
	for(auto itr = cGetRobotData.m_vecRobotsPose.begin(), auto itr2 = cGetRobotData.m_vecRobotVelocity.begin();
		itr!=cGetRobotData.m_vecRobotsPose.end() and itr2 != cGetRobotData.m_vecRobotVelocity.end(); itr++, itr2++) {
		CVector2 temp_var = itr->second - swarm_centroid;
		ang_momentum += itr2->second.CrossProduct(temp_var);
	}
	ang_momentum /= (num_robots*MAX_R);

	/*Get the group rotation*/
	grp_rotation.Set(0.0,0.0);
	for(auto itr = cGetRobotData.m_vecRobotsPose.begin(), auto itr2 = cGetRobotData.m_vecRobotVelocity.begin();
		itr!=cGetRobotData.m_vecRobotsPose.end() and itr2 != cGetRobotData.m_vecRobotVelocity.end(); itr++, itr2++) {
		CVector2 temp_var = itr->second - swarm_centroid;
		if(temp_var.GetX()!=0.0 and temp_var.GetY()!=0.0) {
			temp_var.Normalize();
			ang_momentum += itr2->second.CrossProduct(temp_var);
		}
	}
	grp_rotation /= num_robots;

	std::vector<Real> temp_vec = {avg_speed, scatter, rad_variance, ang_momentum.GetX(), ang_momentum.GetY(),
								  grp_rotation.GetX(), grp_rotation.GetY()};
	
	/*copy it to the feature vector*/
	feature_vector.push_back(temp_vec);
	feature_archive.push_back(temp_vec);

	if(feature_vector.size() == 10) {
		ofs << " more lorem ipsum";
	}

	/*
	call the function for the novelty search here
	*/

	// Get the velocity from the GA
	// BuzzForeachVM(SetRobotVelocity(m_vecVelocity));
}



/****************************************/
/****************************************/

CMPGAExperiment1LoopFunctions::~CMPGAExperiment1LoopFunctions() {
	delete[] m_pfControllerParams;
}

/****************************************/
/****************************************/

void CMPGAExperiment1LoopFunctions::Reset() {
	/*
	* Move robot to the initial position corresponding to the current trial
	*/
	ofs.close();

	if(!MoveEntity(
		m_pcFootBot->GetEmbodiedEntity(),             // move the body of the robot
		m_vecInitSetup[GetTrial()].Position,    // to this position
		m_vecInitSetup[GetTrial()].Orientation, // with this orientation
		false                                         // this is not a check, leave the robot there
		)) {
			LOGERR << "Can't move robot in <"
			<< m_vecInitSetup[GetTrial()].Position
			<< ">, <"
			<< m_vecInitSetup[GetTrial()].Orientation
			<< ">"
			<< std::endl;
		}
}

/****************************************/
/****************************************/

void CMPGAExperiment1LoopFunctions::ConfigureFromGenome(const Real* pf_genome) {
	/* Copy the genes into the NN parameter buffer */
	for(size_t i = 0; i < GENOME_SIZE; ++i) {
		m_pfControllerParams[i] = pf_genome[i];
	}
	/* Set the NN parameters */
	// m_pcController->GetPerceptron().SetOnlineParameters(GENOME_SIZE, m_pfControllerParams);
}

/****************************************/
/****************************************/

std::vector<Real> CMPGAExperiment1LoopFunctions::Score() {
	/* The score of a robot is its feature vector */
	return feature_vector;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CMPGAExperiment1LoopFunctions, "experiment1_loop_functions")
