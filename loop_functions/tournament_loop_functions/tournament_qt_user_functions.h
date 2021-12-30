#ifndef FORAGING_QT_USER_FUNCTIONS_H
#define FORAGING_QT_USER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

using namespace argos;

class CTournamentLoopFunctions;

class CTournamentQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CTournamentQTUserFunctions();

   virtual ~CTournamentQTUserFunctions() {}

   void Draw(CFootBotEntity& c_entity);
   virtual void DrawInWorld();

private:
    CTournamentLoopFunctions& m_cTrajLF;
   
};

#endif
