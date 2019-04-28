#ifndef MPGA_PHOTOTAXIS_LOOP_FUNCTIONS_H
#define MPGA_PHOTOTAXIS_LOOP_FUNCTIONS_H

/* The NN controller */
//#include <controllers/footbot_nn/footbot_nn_controller.h>

/* ARGoS-related headers */
#include <buzz/argos/buzz_loop_functions.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include "buzz/buzzvm.h"
#include <loop_functions/mpga_loop_functions.h>

/****************************************/
/****************************************/

/*
* The size of the genome.
* 
* The genome is the set of NN weights. The NN is a simple
* 2-layer perceptron. The inputs are 24 proximity readings and
* 24 light readings. The outputs are 2 wheels speeds. The total
* number of weights is therefore:
*
* W = (I + 1) * O = (24 + 24 + 1) * 2 = 98
*
* where:
*   W = number of weights
*   I = number of inputs
*   O = number of outputs
*/
static const size_t GENOME_SIZE = 40;
static const int VEC_2D_SIZE = 2;
/****************************************/
/****************************************/

using namespace argos;

class CMPGAExperiment1LoopFunctions : public CMPGALoopFunctions {

public:

	CMPGAExperiment1LoopFunctions();
	virtual ~CMPGAExperiment1LoopFunctions();

	virtual void Init(TConfigurationNode& t_tree);
	virtual void Reset();

	/* Configures the robot controller from the genome */
	virtual void ConfigureFromGenome(const Real* pf_genome);

	/* Calculates the performance of the robot in a trial */
	virtual Real Score();
	virtual void PostStep();

	inline int GetNumRobots() const;
	void CalculateNovelty();
   virtual bool IsExperimentFinished();

   /**
    * Executes user-defined destruction logic.
    * This method should undo whatever is done in Init().
    * @see Init()
    */
   virtual void Destroy();



	Real avg_speed;
	Real scatter;
	Real ang_momentum;
	Real grp_rotation;
	Real rad_variance;
	CVector2 swarm_centroid;
	int num_robots, time_step;
	Real swarm_score;

	std::vector<std::vector<Real> > feature_archive, feature_vector;
	std::vector<float> l_wheel, r_wheel;

	CRandom::CRNG* m_pcRNG;
	std::ofstream ofs;
	std::string m_strOutFile;

private:

	/* The initial setup of a trial */
	struct SInitSetup {
	  CVector3 Position;
	  CQuaternion Orientation;
	};

	std::vector<SInitSetup> m_vecInitSetup;
	CFootBotEntity* m_pcFootBot;
	// CFootBotNNController* m_pcController;
	Real* m_pfControllerParams;
	// std::ofstream ofs;
};

#endif
