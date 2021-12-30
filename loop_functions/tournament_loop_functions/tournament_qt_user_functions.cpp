#include "tournament_qt_user_functions.h"
#include <controllers/footbot_foraging/footbot_foraging.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include "tournament_loop_functions.h"


using namespace argos;

/****************************************/
/****************************************/

CTournamentQTUserFunctions::CTournamentQTUserFunctions() :
        m_cTrajLF(dynamic_cast<CTournamentLoopFunctions&>(CSimulator::GetInstance().GetLoopFunctions())) {
   RegisterUserFunction<CTournamentQTUserFunctions,CFootBotEntity>(&CTournamentQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void CTournamentQTUserFunctions::Draw(CFootBotEntity& c_entity) {
    CFootBotForaging& cController = reinterpret_cast<CFootBotForaging&>(c_entity.GetControllableEntity().GetController());

    // was -dynamic_cast- change to -reinterpret_cast-
   CFootBotForaging::SFoodData& sFoodData = cController.GetFoodData();
   if(sFoodData.HasFoodItem) {
      DrawCylinder(
         CVector3(0.0f, 0.0f, 0.3f),
         CQuaternion(),
         0.1f,
         0.05f,
         CColor::BLACK);
   }
}

void CTournamentQTUserFunctions::DrawInWorld() {
    DrawText(CVector3(m_cTrajLF.getFirstBase().GetX(), m_cTrajLF.getFirstBase().GetY(), 0.3),   // position
             m_cTrajLF.getFirstColor()+" base"); // text
    DrawText(CVector3(m_cTrajLF.getSecondBase().GetX(), m_cTrajLF.getSecondBase().GetY(), 0.3),   // position
             m_cTrajLF.getSecondColor()+" base"); // text
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CTournamentQTUserFunctions, "tournament_qt_user_functions")
