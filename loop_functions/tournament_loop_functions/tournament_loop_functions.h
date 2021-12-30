#ifndef FORAGING_LOOP_FUNCTIONS_H
#define FORAGING_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>


using namespace argos;


class CTournamentLoopFunctions : public CLoopFunctions {

public:

   CTournamentLoopFunctions();
   virtual ~CTournamentLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_tree);
   virtual void Reset();
   virtual void Destroy();
   virtual CColor GetFloorColor(const CVector2& c_position_on_plane);
   virtual void PreStep();
   virtual void PostExperiment();

   CVector2 getFirstBase() {return first_base;}
    CVector2 getSecondBase() {return second_base;}
    std::string getFirstColor(){return first_color;}
    std::string getSecondColor(){return second_color;}
    std::string getFirstBaseColor(){return first_base_color;}
    std::string getSecondBaseColor(){return second_base_color;}


private:
    CRange<Real> m_cForagingArenaSideX, m_cForagingArenaSideY;
    Real halfX, halfY;
    int numOfFoodBases;

    int first_id, second_id;
    std::string first_color, second_color;
    std::string first_base_color, second_base_color;
    std::string firstName, secondName;
    CVector2 first_base, second_base;
    bool firstTimeFirstTeam = true, firstTimeSecondTeam = true;
    int first_score = 0, second_score = 0;

    std::string arenaName;
    int seed;
   Real m_fFoodSquareRadius;
    Real m_fHomeSquareRadius;
   CRange<Real> m_cTournamentArenaSideX, m_cTournamentArenaSideY;
   std::vector<CVector2> m_cFoodPos;
   CFloorEntity* m_pcFloor;
   CRandom::CRNG* m_pcRNG;

   std::string m_strOutput;
   std::ofstream m_cOutput;

   UInt32 m_unCollectedFood;
   SInt64 m_nEnergy;
   UInt32 m_unEnergyPerFoodItem;
   UInt32 m_unEnergyPerWalkingRobot;

    CVector2 randLegalFoodLocation();

    bool isFirst = true;
};


#endif
