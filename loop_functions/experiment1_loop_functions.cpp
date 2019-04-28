#include "experiment1_loop_functions.h"

/****************************************/
/****************************************/

static const Real MAX_R = 10.0;


CMPGAExperiment1LoopFunctions::CMPGAExperiment1LoopFunctions() {}


struct GetRobotData : public CBuzzLoopFunctions::COperation {
// struct GetRobotData : public CBuzzLoopFunctions::COperation {
	GetRobotData() {}
	/** The action happens here */
	virtual void operator()(const std::string& str_robot_id,buzzvm_t t_vm) {

	 	/*Get the pose of the robots*/
		BuzzTableOpen(t_vm, "robot_pose");
		buzzobj_t tPose = BuzzGet(t_vm, "robot_pose");
		/* Make sure it's the type we expect (a table) */
		if(!buzzobj_istable(tPose)) {
			LOGERR << str_robot_id << ": variable 'pose.position' has wrong type " << buzztype_desc[tPose->o.type] << std::endl;
			return;
		}
		m_vecRobotsPose[t_vm->robot].resize(VEC_2D_SIZE, 0.0);
		
		/* Get the object */
		for(int i = 0; i < VEC_2D_SIZE; ++i) {
			buzzobj_t tPoseValue = BuzzTableGet(t_vm, i);
			/* Make sure it's the type we expect (a float) */
			if(!buzzobj_isfloat(tPoseValue)) {
				LOGERR << str_robot_id << ": element 'pose[]' has wrong type " << buzztype_desc[tPoseValue->o.type] <<std::endl;
			}
			else {
				/* Get the value */
				float fPoseValue = buzzobj_getfloat(tPoseValue);
				/* Set the mapping */
				m_vecRobotsPose[t_vm->robot][i] = fPoseValue;
			}

			BuzzTableOpen(t_vm, "Velocity");
			buzzobj_t tVelocity = BuzzGet(t_vm, "Velocity");
			if(!buzzobj_istable(tVelocity)) {
				LOGERR << str_robot_id << ": variable 'velocity' has wrong type " << buzztype_desc[tVelocity->o.type] << std::endl;
				return;
			}
		}

		m_vecRobotsVelocity[t_vm->robot].resize(VEC_2D_SIZE, 0.0);
		
		for(int i = 0; i < VEC_2D_SIZE; ++i) {	
			/* Get the object */
			buzzobj_t tVelocityValue = BuzzTableGet(t_vm, i);
			/* Make sure it's the type we expect (a float) */
			if(!buzzobj_isfloat(tVelocityValue)) {
				LOGERR << str_robot_id << ": element 'threshold["  "]' has wrong type " << buzztype_desc[tVelocityValue->o.type] << std::endl;
			}
			else {
				/* Get the value */
				float fVelocityValue = buzzobj_getfloat(tVelocityValue);
				/* Set the mapping */
				m_vecRobotsVelocity[t_vm->robot][i] = fVelocityValue;
			}
		}	

	}

	/*map for pose*/
	std::map<int, std::vector<Real> > m_vecRobotsPose;
	std::map<int, std::vector<Real> > m_vecRobotsVelocity;
};

int ci = 0;

struct SetRobotVelocity : public CBuzzLoopFunctions::COperation {

	/** Constructor */
	SetRobotVelocity(const std::vector<float> l_vel, const std::vector<float> r_vel) : m_pcLeftVel(l_vel), m_pcRightVel(r_vel) {}

	/** The action happens here */
	virtual void operator()(const std::string& str_robot_id,buzzvm_t t_vm) {
		// /* Set the values of the table 'stimulus' in the Buzz VM */
		// int i=0;
		BuzzTableOpen(t_vm, "control_input");
		BuzzTablePut(t_vm, 0, m_pcLeftVel[ci]);
		BuzzTablePut(t_vm, 1, m_pcRightVel[ci]);
		ci++;
		if(ci == GENOME_SIZE)
			ci = 0;
		BuzzTableClose(t_vm);
	}
	/** Calculated stimuli */
	const std::vector<float> m_pcLeftVel, m_pcRightVel;
};

inline int CMPGAExperiment1LoopFunctions::GetNumRobots() const {
	return m_mapBuzzVMs.size();
}

/****************************************/
/****************************************/

void CMPGAExperiment1LoopFunctions::Init(TConfigurationNode& t_tree) {
	/*
	* Create the random number generator
	*/
	CBuzzLoopFunctions::Init(t_tree);
	GetNodeAttribute(t_tree, "outfile", m_strOutFile);
	ofs.open (m_strOutFile, std::ofstream::out | std::ofstream::app);

	// m_vecInitSetup = 5;
	// m_pcFootBot(NULL),
   m_pfControllerParams = new Real[GENOME_SIZE];
   avg_speed = 0.0;
   scatter = 0.0;
   ang_momentum = 0.0;
   grp_rotation = 0.0;
   rad_variance = 0.0;
   swarm_centroid = CVector2(0.0,0.0);
   
   

	m_pcRNG = CRandom::CreateRNG("argos");
	swarm_score = 0.0;

	/*
	* Create the foot-bot and get a reference to its controller
	*/
	// m_pcFootBot = new CFootBotEntity(
	// 	"fb",    // entity id
	// 	"fnn"    // controller id as set in the XML
	// );
	// AddEntity(*m_pcFootBot);
	// // m_pcController = &dynamic_cast<CFootBotNNController&>(m_pcFootBot->GetControllableEntity().GetController());

	/*
	* Create the initial setup for each trial
	* The robot is placed 4.5 meters away from the light
	* (which is in the origin) at angles
	* { PI/12, 2*PI/12, 3*PI/12, 4*PI/12, 5*PI/12 }
	* wrt to the world reference.
	* Also, the rotation of the robot is chosen at random
	* from a uniform distribution.
	*/
	// CRadians cOrient;
	// for(size_t i = 0; i < 5; ++i) {
	// 	/* Set position */
	// 	m_vecInitSetup[i].Position.FromSphericalCoords(
	// 		4.5f,                                          // distance from origin
	// 		CRadians::PI_OVER_TWO,                         // angle with Z axis
	// 		static_cast<Real>(i+1) * CRadians::PI / 12.0f // rotation around Z
	// 	);
	// 	/* Set orientation */
	// 	cOrient = m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE);
	// 	m_vecInitSetup[i].Orientation.FromEulerAngles(
	// 		cOrient,        // rotation around Z
	// 		CRadians::ZERO, // rotation around Y
	// 		CRadians::ZERO  // rotation around X
	// 	);
	// }

	/*
	* Process trial information, if any
	*/
	try {
		UInt32 unTrial;
		GetNodeAttribute(t_tree, "trial", unTrial);
		SetTrial(unTrial);
		Reset();
	}
	catch(CARGoSException& ex) {}
}


void CMPGAExperiment1LoopFunctions::PostStep() {
	
	GetRobotData cGetRobotData;
	BuzzForeachVM(cGetRobotData);
	int num_robots = GetNumRobots();
	/*Get swarm centroid*/
	Real avgX = 0.0, avgY = 0.0;
	for(auto itr = cGetRobotData.m_vecRobotsPose.begin(); itr!=cGetRobotData.m_vecRobotsPose.end(); itr++) {
		avgX += itr->second[0];
		avgY += itr->second[1];
	}
	swarm_centroid.SetX(avgX/num_robots);
	swarm_centroid.SetY(avgY/num_robots);

	/*Get the average velocity of the swarm*/
	Real avg_speedx = 0.0, avg_speedy = 0.0;
	for(auto itr = cGetRobotData.m_vecRobotsVelocity.begin(); itr!=cGetRobotData.m_vecRobotsVelocity.end(); itr++) {
		avg_speed  += std::sqrt(itr->second[0]*itr->second[0] + itr->second[1]*itr->second[1]);
	}
	avg_speed = avg_speed/num_robots*0.01;
	   // 	std::vector<Real> m_pVecAvgSpeed;
   	// std::vector<Real> m_pVecScatter;
   	// std::vector<Real> m_pVecAngMomentum;
   	// std::vector<Real> m_pVecGrpRotation;
   	// std::vector<Real> m_pVecRadVariance;

   	m_pVecAvgSpeed.push_back(avg_speed);


	/*Get the scatter for the swarm*/
	scatter = 0.0;
	for(auto itr = cGetRobotData.m_vecRobotsPose.begin(); itr!=cGetRobotData.m_vecRobotsPose.end(); itr++) {
		scatter += std::pow(itr->second[0] -  swarm_centroid.GetX(), 2) + std::pow(itr->second[1] -  swarm_centroid.GetY(), 2);
	}
	scatter /= (MAX_R*MAX_R*num_robots);

	m_pVecScatter.push_back(scatter);

	/*Get the radial variance for the swarm*/
	rad_variance = 0.0;
	Real variance = 0.0;
	for(auto itr = cGetRobotData.m_vecRobotsPose.begin(); itr!=cGetRobotData.m_vecRobotsPose.end(); itr++) {
		variance += std::sqrt(std::pow(itr->second[0] -  swarm_centroid.GetX(), 2) + std::pow(itr->second[1] -  swarm_centroid.GetY(), 2));
	}
	variance /= (std::pow(MAX_R*num_robots,2));
	rad_variance = scatter - variance;

	m_pVecRadVariance.push_back(rad_variance);

	/*Get the angular momentum*/
	ang_momentum = 0.0;
	// auto itr = cGetRobotData.m_vecRobotsPose.begin();
	for(auto itr = cGetRobotData.m_vecRobotsPose.begin(), itr2 = cGetRobotData.m_vecRobotsVelocity.begin();
		itr!=cGetRobotData.m_vecRobotsPose.end() and itr2 != cGetRobotData.m_vecRobotsVelocity.end(); itr++, itr2++) {
		CVector2 temp_pose = CVector2(itr->second[0], itr->second[1]);
		CVector2 temp_vel = CVector2(itr2->second[0], itr2->second[1]);
		CVector2 temp_var = temp_pose - swarm_centroid;
		ang_momentum = ang_momentum + temp_vel.CrossProduct(temp_var);
	}
	ang_momentum /= (num_robots*MAX_R);

	m_pVecAngMomentum.push_back(ang_momentum);

	/*Get the group rotation*/
	grp_rotation = 0.0;
	for(auto itr = cGetRobotData.m_vecRobotsPose.begin(), itr2 = cGetRobotData.m_vecRobotsVelocity.begin();
		itr!=cGetRobotData.m_vecRobotsPose.end() and itr2 != cGetRobotData.m_vecRobotsVelocity.end(); itr++, itr2++) {
		CVector2 temp_pose = CVector2(itr->second[0], itr->second[1]);
		CVector2 temp_vel = CVector2(itr2->second[0], itr2->second[1]);
		CVector2 temp_var = temp_pose - swarm_centroid;
		if(temp_var.GetX()!=0.0 and temp_var.GetY()!=0.0) {
			temp_var.Normalize();
			grp_rotation += temp_vel.CrossProduct(temp_var);
		}
	}
	grp_rotation /= num_robots;

	m_pVecGrpRotation.push_back(grp_rotation);

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

	// if(!MoveEntity(
	// 	m_pcFootBot->GetEmbodiedEntity(),             // move the body of the robot
	// 	m_vecInitSetup[GetTrial()].Position,    // to this position
	// 	m_vecInitSetup[GetTrial()].Orientation, // with this orientation
	// 	false                                         // this is not a check, leave the robot there
	// 	)) {
	// 		LOGERR << "Can't move robot in <"
	// 		<< m_vecInitSetup[GetTrial()].Position
	// 		<< ">, <"
	// 		<< m_vecInitSetup[GetTrial()].Orientation
	// 		<< ">"
	// 		<< std::endl;
	// 	}
}

/****************************************/
/****************************************/

void CMPGAExperiment1LoopFunctions::ConfigureFromGenome(const Real* pf_genome) {
	/* Copy the genes into the NN parameter buffer */
	for(size_t i = 0; i < GENOME_SIZE; ++i) {
		m_pfControllerParams[i] = pf_genome[i];
	}

	if(l_wheel.empty() and r_wheel.empty()) {
		for(int i=0; i<GENOME_SIZE; i++) {
			if(i%2 == 0) 
				l_wheel.push_back(m_pfControllerParams[i]);
			else
				r_wheel.push_back(m_pfControllerParams[i]);
		}
	}

	else {
		for(int i=0; i<GENOME_SIZE; i++) {
			if(i%2 == 0) 
				l_wheel.at(i/2) = m_pfControllerParams[i];
			else
				r_wheel.at(i/2) = m_pfControllerParams[i];
		}
	}

	for(int i=0; i<5; i++) {
		LOG<<"l: "<<l_wheel.at(i)<<"		"<<r_wheel.at(i)<<std::endl;
	}

	BuzzForeachVM(SetRobotVelocity(l_wheel, r_wheel));

	/* Set the NN parameters */
	// m_pcController->GetPerceptron().SetOnlineParameters(GENOME_SIZE, m_pfControllerParams);
}

/****************************************/
/****************************************/

Real CMPGAExperiment1LoopFunctions::Score() {
	/* The performance is simply the distance of the robot to the origin */
	// return m_pcFootBot->GetEmbodiedEntity().GetOriginAnchor().Position.Length();

	// std::vector<Real> temp_vec = {avg_speed, scatter, rad_variance, ang_momentum, grp_rotation};
	/*copy it to the feature vector*/
	// feature_vector.push_back(temp_vec);

	avg_speed = 0.0; scatter = 0.0; rad_variance = 0.0; ang_momentum = 0.0; grp_rotation = 0.0;
	for(int i=0; i<m_pVecScatter.size(); i++) {
		avg_speed += m_pVecAvgSpeed.at(i);
		scatter += m_pVecScatter.at(i);
		rad_variance += m_pVecRadVariance.at(i);
		ang_momentum += m_pVecAngMomentum.at(i);
		grp_rotation += m_pVecGrpRotation.at(i);
	}


	avg_speed /= m_pVecScatter.size();
	scatter /= m_pVecScatter.size();
	rad_variance /= m_pVecScatter.size();
	ang_momentum /= m_pVecScatter.size();
	grp_rotation /= m_pVecScatter.size();
	
	/*write code for fscore*/
	swarm_score = ang_momentum/scatter;
	return swarm_score;
}

void CMPGAExperiment1LoopFunctions::Destroy() {

}

bool CMPGAExperiment1LoopFunctions::IsExperimentFinished() {
   /* Feel free to try out custom ending conditions */
   return false;
}


/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CMPGAExperiment1LoopFunctions, "experiment1_loop_functions")
