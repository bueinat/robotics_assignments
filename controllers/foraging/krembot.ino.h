#include <Krembot/controller/krembot_controller.h>

struct ForagingMsg{
    std::string ourColor;
    std::string opponentColor;
    CVector2 homeLocation;
    std::string ourBaseColor;
    std::string opponentBaseColor;
};

struct PosMsg{
    CVector2 pos;
    CDegrees degreeX;
};

class foraging_controller : public KrembotController {
protected:
    bool isFirst = true;

    void writeTeamColor()
    {
        if (foragingMsg.ourColor.compare("green") == 0)
        {
            krembot.Led.write(0,255,0);
        }
        else if (foragingMsg.ourColor.compare("blue") == 0)
        {
            krembot.Led.write(0,0,255);
        }
        else if (foragingMsg.ourColor.compare("red") == 0)
        {
            krembot.Led.write(255,0,0);
        }
        else
        {
            throw "FAIL to write team color";
        }
    }
public:
    ForagingMsg foragingMsg;
    PosMsg posMsg;
    bool isWander = true;
    bool hasFood = false;
    int teamID;
    std::string teamName;

    ParticleObserver Particle;
    ~foraging_controller() = default;
    virtual void setup() = 0;
    virtual void loop() = 0;

    void Init(TConfigurationNode &t_node) override {
        KrembotController::Init(t_node);
        if ( ! krembot.isInitialized() ) {
            throw std::runtime_error("krembot.ino.cpp: krembot wasn't initialized in controller");
        }
        GetNodeAttribute(t_node, "teamID", teamID);
        Particle.setName(krembot.getName());
    }
    void ControlStep() override {
        if(isFirst) {
            setup();
            isFirst = false;
        }
        loop();
    }
};


//REGISTER_CONTROLLER(foraging_controller, "foraging_controller")
