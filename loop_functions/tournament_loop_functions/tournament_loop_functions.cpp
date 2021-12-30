#include "tournament_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <controllers/footbot_foraging/footbot_foraging.h>
#include <Krembot/controller/krembot_controller.h>
//#include <argos3/core/simulator/entity/composable_entity.h>
//#include "Krembot/krembot.h"
#include <controllers/foraging/krembot.ino.h>

/****************************************/
/****************************************/

CTournamentLoopFunctions::CTournamentLoopFunctions() :
        m_pcFloor(NULL),
        m_pcRNG(NULL){
}

/****************************************/
/****************************************/

void CTournamentLoopFunctions::Init(TConfigurationNode& t_node) {
    try {

        TConfigurationNode& tTournament = GetNode(t_node, "tournament");
        /* Get a pointer to the floor entity */
        m_pcFloor = &GetSpace().GetFloorEntity();
        /* Get the number of food items we want to be scattered from XML */
        GetNodeAttribute(tTournament, "halfX", halfX);
        GetNodeAttribute(tTournament, "halfY", halfY);

        /* Create a new RNG */
        m_pcRNG = CRandom::CreateRNG("argos");
        seed = CRandom::GetSeedOf("argos");
        GetNodeAttribute(tTournament, "arenaName", arenaName);

        m_cForagingArenaSideX = CRange<Real>(-halfX, halfX);
        m_cForagingArenaSideY = CRange<Real>(-halfY, halfY);

        GetNodeAttribute(tTournament, "first_id", first_id);
        GetNodeAttribute(tTournament, "first_color", first_color);
        GetNodeAttribute(tTournament, "first_base", first_base);
        GetNodeAttribute(tTournament, "first_base_color", first_base_color);

        GetNodeAttribute(tTournament, "second_id", second_id);
        GetNodeAttribute(tTournament, "second_color", second_color);
        GetNodeAttribute(tTournament, "second_base", second_base);
        GetNodeAttribute(tTournament, "second_base_color", second_base_color);
    

        CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");

        for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
            it != m_cFootbots.end();
            ++it) {

            /* Get handle to foot-bot entity and controller */
            CFootBotEntity &cFootBot = *any_cast<CFootBotEntity *>(it->second);
            foraging_controller &fController = dynamic_cast<foraging_controller &>(cFootBot.GetControllableEntity().GetController());
            if (fController.teamID == first_id)
            {
                fController.foragingMsg.ourColor = first_color;
                fController.foragingMsg.opponentColor = second_color;
                fController.foragingMsg.homeLocation = first_base;
                fController.foragingMsg.ourBaseColor = first_base_color;
                fController.foragingMsg.opponentBaseColor = second_base_color;
            }
            else if (fController.teamID == second_id)
            {
                fController.foragingMsg.ourColor = second_color;
                fController.foragingMsg.opponentColor = first_color;
                fController.foragingMsg.homeLocation = second_base;
                fController.foragingMsg.ourBaseColor = second_base_color;
                fController.foragingMsg.opponentBaseColor = first_base_color;
            }
            else
            {
                throw "FAIL - problem with teamID configuration";
            }
        }

        GetNodeAttribute(tTournament, "number_of_food_bases", numOfFoodBases);
        /* Get the number of food items we want to be scattered from XML */
        GetNodeAttribute(tTournament, "food_radius", m_fFoodSquareRadius);
        GetNodeAttribute(tTournament, "home_radius", m_fHomeSquareRadius);
        m_fHomeSquareRadius *= m_fHomeSquareRadius;
        m_fFoodSquareRadius *= m_fFoodSquareRadius;
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
    }
}

/****************************************/
/****************************************/

void CTournamentLoopFunctions::Reset() {

}

/****************************************/
/****************************************/

void CTournamentLoopFunctions::Destroy() {

}

/****************************************/
/****************************************/

CColor CTournamentLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) {
//   if(c_position_on_plane.GetX() < -1.0f) {
//      return CColor::GRAY50;
//   }
    if (isFirst)
    {
        return CColor::WHITE;
    } else {
        for (UInt32 i = 0; i < numOfFoodBases; ++i) {
            if ((c_position_on_plane - m_cFoodPos[i]).SquareLength() < m_fFoodSquareRadius) {
                return CColor::BLACK;
            }
        }
        return CColor::WHITE;
    }
}

/****************************************/
/****************************************/

void CTournamentLoopFunctions::PreStep() {

    if(isFirst)
    {
        for(int i = 0 ; i< numOfFoodBases; i++)
        {
            CVector2 foodLoc = randLegalFoodLocation();
            m_cFoodPos.push_back(
                    CVector2(foodLoc.GetX(),
                             foodLoc.GetY()));
        }
        isFirst = false;
        m_pcFloor->SetChanged();
    }

    /* Logic to pick and drop food items */
    /*
     * If a robot is in the nest, drop the food item
     * If a robot is on a food item, pick it
     * Each robot can carry only one food item per time
     *
    /* Check whether a robot is on a food item */
    CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");

    for(CSpace::TMapPerType::iterator it = m_cFootbots.begin();
        it != m_cFootbots.end();
        ++it) {

        /* Get handle to foot-bot entity and controller */
        CFootBotEntity& cFootBot = *any_cast<CFootBotEntity*>(it->second);
        CFootBotForaging& cController = reinterpret_cast<CFootBotForaging&>(cFootBot.GetControllableEntity().GetController());
        foraging_controller& fController = dynamic_cast<foraging_controller&>(cFootBot.GetControllableEntity().GetController());

        if (fController.teamID == first_id)
        {
            firstName = fController.teamName;
            firstTimeFirstTeam = false;
        }
        else if (fController.teamID == second_id)
        {
            secondName = fController.teamName;
            firstTimeSecondTeam = false;
        }

        /* Get the position of the foot-bot on the ground as a CVector2 */
        CVector2 cPos;
        cPos.Set(cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                 cFootBot.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
        Real orientation = cFootBot.GetEmbodiedEntity().GetOriginAnchor().Orientation.GetZ();
//      LOG << orientation << "\n";
        // from pos to ind
        fController.posMsg.pos = cPos;
        CQuaternion cQuat = cFootBot.GetEmbodiedEntity().GetOriginAnchor().Orientation;
        CRadians cZAngle, cYAngle, cXAngle;
        CDegrees xDegree;
        cQuat.ToEulerAngles(cZAngle, cYAngle, cXAngle);
        xDegree = ToDegrees(cZAngle);
        fController.posMsg.degreeX = xDegree;


        /* The foot-bot has a food item */
        CFootBotForaging::SFoodData& sFoodData = cController.GetFoodData();
        if(fController.hasFood) {
            /* Check whether the foot-bot is in the nest */
            if (((fController.teamID == first_id) && (cPos - first_base).SquareLength() < m_fHomeSquareRadius)
                ||((fController.teamID == second_id) && (cPos - second_base).SquareLength() < m_fHomeSquareRadius))
            {
                /* Place a new food item on the ground */
                CVector2 foodLoc = randLegalFoodLocation();
                m_cFoodPos[sFoodData.FoodItemIdx].Set(foodLoc.GetX(),
                                                      foodLoc.GetY());
                /* Drop the food item */
                fController.hasFood = false;
                fController.isWander = true;

                if(fController.teamID == first_id)
                {
                    first_score += 1;
                }
                else if (fController.teamID == second_id)
                {
                    second_score += 1;
                }

                //for qt
                sFoodData.HasFoodItem = false;
                sFoodData.FoodItemIdx = 0;
                ++sFoodData.TotalFoodItems;

                m_pcFloor->SetChanged();
                break;
            }
        }
        else{
            /* The foot-bot has no food item */
            /* Check whether the foot-bot is out of the nest */
            /* Check whether the foot-bot is on a food item */
            bool bDone = false;
            for(size_t i = 0; i < numOfFoodBases && !bDone; ++i) {
                if((cPos - m_cFoodPos[i]).SquareLength() < m_fFoodSquareRadius) {
                    /* If so, we move that item out of sight */
                  m_cFoodPos[i].Set(100.0f, 100.f);
                    /* The foot-bot is now carrying an item */
                    fController.hasFood = true;
                    fController.isWander = false;

                    //for qt
                    sFoodData.HasFoodItem = true;
                    sFoodData.FoodItemIdx = i;

                    /* The floor texture must be updated */
                    m_pcFloor->SetChanged();
                    /* We are done */
                    bDone = true;
                }
            }
        }
    }
}

CVector2 CTournamentLoopFunctions::randLegalFoodLocation()
{
    CVector2 foodLoc;
    bool done = false;
    while(!done)
    {
        foodLoc.Set(m_pcRNG->Uniform(m_cForagingArenaSideX),
             m_pcRNG->Uniform(m_cForagingArenaSideY));

        SBoundingBox bb;
        bb.MinCorner.Set(foodLoc.GetX() - m_fFoodSquareRadius, foodLoc.GetY() - m_fFoodSquareRadius, 0);
        bb.MaxCorner.Set(foodLoc.GetX() + m_fFoodSquareRadius, foodLoc.GetY() + m_fFoodSquareRadius, 0.5);

        CSpace::TMapPerType& m_boxs = GetSpace().GetEntitiesByType("box");
        for(CSpace::TMapPerType::iterator it = m_boxs.begin();
            it != m_boxs.end();
            ++it) {

            CBoxEntity& box = *any_cast<CBoxEntity*>(it->second);
            if (!box.GetEmbodiedEntity().GetBoundingBox().Intersects(bb))
            {
                done = true;
            }
        }
    }

    return foodLoc;
}

void CTournamentLoopFunctions::PostExperiment() {
    // m_cOutput.open("scores.txt", std::ios_base::trunc | std::ios_base::out);
//    m_cOutput.open("/home/edenha/krembot_ws/tournament/scores.txt", std::ios_base::trunc | std::ios_base::out);
    m_cOutput.open("/home/einat/scores.txt", std::ios_base::trunc | std::ios_base::out);
    m_cOutput << "arena\tseed\ttime\tfirst-name\t\tfirst-score\tsecond-name\t\tsecond-score\twinner" << std::endl;
    std::string winner;
    if (first_score == second_score)
    {
        winner = "tie";
    }
    else
    {
        winner = (first_score > second_score) ? (firstName + " (first team)") : (secondName + " (second team)");
    }
    m_cOutput << arenaName << "\t"
              << seed << "\t"
              << GetSpace().GetSimulationClock() << "\t"
              << firstName << "\t" << "\t"
              << first_score << "\t"
              << secondName << "\t" << "\t"
              << second_score << "\t"
              << winner << std::endl;

    m_cOutput.close();
}


/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CTournamentLoopFunctions, "tournament_loop_functions")
