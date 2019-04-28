#include "novel_cmpga.h"
NovelCMPGA::NovelCMPGA(const CRange<Real>& c_allele_range,
             UInt32 un_genome_size,
             UInt32 un_pop_size,
             Real f_mutation_prob,
             UInt32 un_num_trials,
             UInt32 un_generations,
             bool b_maximize,
             const std::string& str_argosconf,
             TScoreAggregator t_score_aggregator,
             UInt32 un_random_seed) :
	CMPGA(c_allele_range,
             un_genome_size,
             un_pop_size,
             f_mutation_prob,
             un_num_trials,
             un_generations,
             b_maximize,
             str_argosconf,
             t_score_aggregator,
             un_random_seed){}
//extend the parent constructor and maybe do more

void NovelCMPGA::LaunchARGoS(UInt32 un_slave_id) {
   /* Set the global GA instance pointer for signal handler */
   GA_INSTANCE = this;
   /* Install handler for SIGTERM */
   ::signal(SIGTERM, SlaveHandleSIGTERM);
   /* Initialize ARGoS */
   /* Redirect LOG and LOGERR to dedicated files to prevent clutter on the screen */
   std::ofstream cLOGFile(std::string("ARGoS_LOG_" + ToString(::getpid())).c_str(), std::ios::out);
   LOG.DisableColoredOutput();
   LOG.GetStream().rdbuf(cLOGFile.rdbuf());
   std::ofstream cLOGERRFile(std::string("ARGoS_LOGERR_" + ToString(::getpid())).c_str(), std::ios::out);
   LOGERR.DisableColoredOutput();
   LOGERR.GetStream().rdbuf(cLOGERRFile.rdbuf());
   /* The CSimulator class of ARGoS is a singleton. Therefore, to
    * manipulate an ARGoS experiment, it is enough to get its instance */
   argos::CSimulator& cSimulator = argos::CSimulator::GetInstance();
   try {
      /* Set the .argos configuration file
       * This is a relative path which assumed that you launch the executable
       * from argos3-examples (as said also in the README) */
      cSimulator.SetExperimentFileName(m_strARGoSConf);
      /* Load it to configure ARGoS */
      cSimulator.LoadExperiment();
      LOG.Flush();
      LOGERR.Flush();
   }
   catch(CARGoSException& ex) {
      LOGERR << ex.what() << std::endl;
      ::raise(SIGTERM);
   }
   /* Get a reference to the loop functions */
   CMPGALoopFunctions& cLoopFunctions = dynamic_cast<CMPGALoopFunctions&>(cSimulator.GetLoopFunctions());
   /* Create vector of scores */
   std::vector<vector<Real>> vecScores(m_unNumTrials, std::vector());
   /* Continue working until killed by parent */
   while(1) {
      /* Suspend yourself, waiting for parent's resume signal */
      ::raise(SIGTSTP);
      /* Resumed */
      /* Configure the controller with the genome */
      cLoopFunctions.ConfigureFromGenome(m_pcSharedMem->GetGenome(un_slave_id));
      /* Run the trials */
      for(size_t i = 0; i < m_unNumTrials; ++i) {
         /* Tell the loop functions to get ready for the i-th trial */
         cLoopFunctions.SetTrial(i);
         /* Reset the experiment.
          * This internally calls also CMPGALoopFunctions::Reset(). */
         cSimulator.Reset();
         /* Run the experiment */
         cSimulator.Execute();
         /* Store score */
         vecScores[i] = cLoopFunctions.Score();
         LOG.Flush();
         LOGERR.Flush();
      }
      ;
      /* Put result in shared memory */
      m_pcSharedMem->SetScore(un_slave_id, m_tScoreAggregator(vecScores));
   }
}


void NovelCMPGA::Evaluate() {
   /* Set parameters for the processes and resume them */
   for(UInt32 i = 0; i < m_unPopSize; ++i) {
      /* Set genome */
      m_pcSharedMem->SetGenome(i, &(m_tPopulation[i]->Genome[0]));
      /* Resume process */
      ::kill(SlavePIDs[i], SIGCONT);
   }
   /* Wait for all the slaves to finish the run */
   UInt32 unTrialsLeft = m_unPopSize;
   int nSlaveInfo;
   pid_t tSlavePID;
   while(unTrialsLeft > 0) {
      /* Wait for next slave to finish */
      tSlavePID = ::waitpid(-1, &nSlaveInfo, WUNTRACED);
      /* Make sure the slave went back to sleep and didn't crash */
      if(!WIFSTOPPED(nSlaveInfo)) {
         LOGERR << "[FATAL] Slave process with PID " << tSlavePID << " exited, can't continue. Check file ARGoS_LOGERR_" << tSlavePID << " for more information." << std::endl;
         LOG.Flush();
         LOGERR.Flush();
         Cleanup();
         ::exit(1);
      }
      /* All OK, one less slave to wait for */
      --unTrialsLeft;
   }
}
