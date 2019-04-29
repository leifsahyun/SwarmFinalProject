#include "experiment1_loop_functions.h"

/****************************************/
/****************************************/

static const Real MAX_R = 500.0*1.414;


CMPGAExperiment1LoopFunctions::CMPGAExperiment1LoopFunctions():m_vecInitSetup(NUM_ROBOTS) {}


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
		}
		
		/*Velocity measurements*/
		BuzzTableOpen(t_vm, "Velocity");
		buzzobj_t tVelocity = BuzzGet(t_vm, "Velocity");
		if(!buzzobj_istable(tVelocity)) {
			LOGERR << str_robot_id << ": variable 'velocity' has wrong type " << buzztype_desc[tVelocity->o.type] << std::endl;
			return;
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

		/* State measurements */
		BuzzTableOpen(t_vm, "state_vec");
		buzzobj_t tState = BuzzGet(t_vm, "state_vec");
		/* Make sure it's the type we expect (a table) */
		if(!buzzobj_istable(tState)) {
			LOGERR << str_robot_id << ": variable 'state_vec' has wrong type " << buzztype_desc[tState->o.type] << std::endl;
			return;
		}
		m_vecRobotsState[t_vm->robot].resize(3, 0.0);
		
		for(int i = 0; i < 3; ++i) {
			buzzobj_t tStateValue = BuzzTableGet(t_vm, i);
			/* Make sure it's the type we expect (a float) */
			if(!buzzobj_isfloat(tStateValue)) {
				LOGERR << str_robot_id << ": element 'state_vec[]' has wrong type " << buzztype_desc[tStateValue->o.type] <<std::endl;
			}
			else {
				/* Get the value */
				float fStateValue = buzzobj_getfloat(tStateValue);
				/* Set the mapping */
				m_vecRobotsState[t_vm->robot][i] = fStateValue;
			}
		}

	}

	/*maps for pose, velocity, and robot state*/
	std::map<int, std::vector<Real> > m_vecRobotsPose;
	std::map<int, std::vector<Real> > m_vecRobotsVelocity;
	/*robot state map contains a vector of homophily, switching frequency, and current state*/
	std::map<int, std::vector<Real>> m_vecRobotsState;
};

struct SetRobotVelocity : public CBuzzLoopFunctions::COperation {

	/** Constructor */
	SetRobotVelocity(const std::vector<float> l_vel, const std::vector<float> r_vel, const std::vector<float> s_prob) : m_pcLeftVel(l_vel), m_pcRightVel(r_vel), m_pcStateProb(s_prob) {}

	/** The action happens here */
	virtual void operator()(const std::string& str_robot_id,buzzvm_t t_vm) {
		// /* Set the values of the table 'control_input' in the Buzz VM */
		// int i=0;
		BuzzTableOpen(t_vm, "control_input");
		for(int ci=0; ci<GENOME_SIZE/3; ci++){
			BuzzTablePut(t_vm, 3*ci, m_pcLeftVel[ci]);
			BuzzTablePut(t_vm, 3*ci+1, m_pcRightVel[ci]);
			BuzzTablePut(t_vm, 3*ci+2, m_pcStateProb[ci]);
		}
		BuzzTableClose(t_vm);
	}
	/** Calculated stimuli */
	const std::vector<float> m_pcLeftVel, m_pcRightVel, m_pcStateProb;
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
	printErr("in Init");
	// CBuzzLoopFunctions::Init(t_tree);
	printErr("in Init");
	GetNodeAttribute(t_tree, "outfile", m_strOutFile);
	// ofs.open (m_strOutFile, std::ofstream::out | std::ofstream::app);

	
	// m_vecInitSetup(NUM_ROBOTS);
	printErr("reszed m_vecInitSetup");
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

    

    CRadians cOrient;
    for(size_t i = 0; i < NUM_ROBOTS; ++i) {
        // CVector3 pos;
    	printErr("in loop for placing the robots");
        m_vecInitSetup[i].Position.FromSphericalCoords(	
                m_pcRNG->Uniform(CRange<Real>(-1.5, 1.5)),
                CRadians::PI_OVER_TWO,
                m_pcRNG->Uniform(CRange<CRadians>(CRadians(-180.0), CRadians(180.0))));
        m_vecInitSetup[i].Position.SetZ(0.0);
		/* Set orientation */
		cOrient = m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE);
		m_vecInitSetup[i].Orientation.FromEulerAngles(
			cOrient,        // rotation around Z
			CRadians::ZERO, // rotation around Y
			CRadians::ZERO  // rotation around X
		);
        /* Create robot */
        m_pcFootBot = new CFootBotEntity("fb" + std::to_string(i),"bcf",m_vecInitSetup[i].Position, m_vecInitSetup[i].Orientation);
        /* Add it to the simulation */
        AddEntity(*m_pcFootBot);
        /* Add it to the internal lists */
        m_pVecFootbot.push_back(m_pcFootBot);
        m_pVecControllers.push_back(&dynamic_cast<CBuzzController&>(m_pcFootBot->GetControllableEntity().GetController()));
    }
    BuzzRegisterVMs();
	printErr("BuzzRegisterVMs done");

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

void CMPGAExperiment1LoopFunctions::PreStep() {
    if(not (l_wheel.empty() or r_wheel.empty()))
    	SendBuzzCommand();
    	printErr("in PreStep");
}

void CMPGAExperiment1LoopFunctions::PostStep() {
	printErr("in PostStep");
	GetRobotData cGetRobotData;
	BuzzForeachVM(cGetRobotData);
	/*Get swarm centroid*/
	Real avgX = 0.0, avgY = 0.0;
	for(auto itr = cGetRobotData.m_vecRobotsPose.begin(); itr!=cGetRobotData.m_vecRobotsPose.end(); itr++) {
		avgX += itr->second[0];
		avgY += itr->second[1];
	}
	swarm_centroid.SetX(avgX/NUM_ROBOTS);
	swarm_centroid.SetY(avgY/NUM_ROBOTS);

	/*Get the average velocity of the swarm*/
	Real avg_speedx = 0.0, avg_speedy = 0.0;
	for(auto itr = cGetRobotData.m_vecRobotsVelocity.begin(); itr!=cGetRobotData.m_vecRobotsVelocity.end(); itr++) {
		avg_speed  += std::sqrt(itr->second[0]*itr->second[0] + itr->second[1]*itr->second[1]);
	}
	avg_speed = avg_speed/NUM_ROBOTS*0.01;
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
	scatter /= (MAX_R*MAX_R*NUM_ROBOTS);

	m_pVecScatter.push_back(scatter);

	/*Get the radial variance for the swarm*/
	rad_variance = 0.0;
	Real variance = 0.0;
	for(auto itr = cGetRobotData.m_vecRobotsPose.begin(); itr!=cGetRobotData.m_vecRobotsPose.end(); itr++) {
		variance += std::sqrt(std::pow(itr->second[0] -  swarm_centroid.GetX(), 2) + std::pow(itr->second[1] -  swarm_centroid.GetY(), 2));
	}
	variance /= (std::pow(MAX_R*NUM_ROBOTS,2));
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
	ang_momentum /= (NUM_ROBOTS*MAX_R*100);

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
	grp_rotation /= (NUM_ROBOTS*100);
	
	m_pVecGrpRotation.push_back(grp_rotation);
	
	/*Get the average homophily, switching frequency, state ratio*/
	homophily = 0.0;
	switching_freq = 0.0;
	Real num_state_0 = 0.0;
	Real num_state_1 = 0.0;
	for(auto itr = cGetRobotData.m_vecRobotsState.begin(); itr!=cGetRobotData.m_vecRobotsState.end(); itr++) {
		homophily += itr->second[0];
		switching_freq += itr->second[1];
		Real state = itr->second[2];
		if(state==0.0){
			num_state_0++;
		}
		else if(state==1.0){
			num_state_1++;
		}
		//LOG << "state measurements: " << itr->second[0] << ", " << itr->second[1] << ", " << itr->second[2] << std::endl;
		//LOG << "total switching: " << switching_freq << std::endl;
		//LOG.Flush();
	}
	homophily /= NUM_ROBOTS;
	switching_freq /= NUM_ROBOTS;
	if(num_state_1>num_state_0)
		state_ratio = num_state_0/(num_state_0+num_state_1);
	else
		state_ratio = num_state_1/(num_state_0+num_state_1);
	
	
	m_pVecHomophily.push_back(homophily);
	m_pVecSwitchingFreq.push_back(switching_freq);
	m_pVecStateRatio.push_back(state_ratio);
	
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
	// ofs.close();

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

	for(size_t i = 0; i < m_pVecFootbot.size(); i++){
        if(!MoveEntity(
                m_pVecFootbot[i]->GetEmbodiedEntity(),        //Move this robot
                m_vecInitSetup[i].Position,          // with this position
                m_vecInitSetup[i].Orientation,       // with this orientation
                false                                         // this is not a check, so actually move the robot back
                )) {
            LOGERR << "Can't move robot kh(" << i << ") in <"
                   << m_vecInitSetup[i].Position
                   << ">, <"
                   << m_vecInitSetup[i].Orientation
                   << ">"
                   << std::endl;
            LOGERR.Flush();
        }
    }
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
			if(i%3 == 0) 
				l_wheel.push_back(m_pfControllerParams[i]);
			else if(i%3 == 1)
				r_wheel.push_back(m_pfControllerParams[i]);
			else
				s_prob.push_back(m_pfControllerParams[i]);
		}
	}

	else {
		for(int i=0; i<GENOME_SIZE; i++) {
			if(i%3 == 0) 
				l_wheel.at(i/3) = m_pfControllerParams[i];
			else if(i%3 == 1)
				r_wheel.at(i/3) = m_pfControllerParams[i];
			else
				s_prob.at(i/3) = m_pfControllerParams[i];
		}
	}

	for(int i=0; i<4; i++) {
		LOG<<"controller: "<<l_wheel.at(i)<<"		"<<r_wheel.at(i)<<"		"<<s_prob.at(i)<<std::endl;
		LOG.Flush();
	}

	SendBuzzCommand();

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
	homophily = 0.0; switching_freq = 0.0; state_ratio = 0.0;
	for(int i=m_pVecScatter.size()-20; i<m_pVecScatter.size(); i++) {
		avg_speed += m_pVecAvgSpeed.at(i);
		scatter += m_pVecScatter.at(i);
		rad_variance += m_pVecRadVariance.at(i);
		ang_momentum += m_pVecAngMomentum.at(i);
		grp_rotation += m_pVecGrpRotation.at(i);
		homophily += m_pVecHomophily.at(i);
		switching_freq += m_pVecSwitchingFreq.at(i);
		state_ratio += m_pVecStateRatio.at(i);
	}

	avg_speed /= m_pVecScatter.size();
	scatter /= m_pVecScatter.size();
	rad_variance /= m_pVecScatter.size();
	ang_momentum /= m_pVecScatter.size();
	grp_rotation /= m_pVecScatter.size();
	homophily /= m_pVecScatter.size();
	switching_freq /= m_pVecScatter.size();
	state_ratio /= m_pVecScatter.size();
	
	/*write code for fscore*/
	swarm_score = 1/scatter;//avg_speed/(scatter*rad_variance*grp_rotation);
	return swarm_score;
}

void CMPGAExperiment1LoopFunctions::Destroy() {

}

void CMPGAExperiment1LoopFunctions::SendBuzzCommand() {
	BuzzForeachVM(SetRobotVelocity(l_wheel, r_wheel, s_prob));
}

bool CMPGAExperiment1LoopFunctions::IsExperimentFinished() {
   /* Feel free to try out custom ending conditions */
   return false;
}

void CMPGAExperiment1LoopFunctions::printErr(std::string in){
    LOGERR << in << std::endl;
	LOGERR.Flush();
}
/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CMPGAExperiment1LoopFunctions, "experiment1_loop_functions")
