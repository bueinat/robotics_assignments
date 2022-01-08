#include <Krembot/controller/krembot_controller.h>
#include "controllers/foraging/krembot.ino.h"


class foraging_4_controller : public foraging_controller {
private:
    enum State{
        turn,
        move,
        stop
    };

    State state = move;
    SandTimer sandTimer;
    int turning_speed = 100;
    int direction = 1;
    Colors ourColor, opponentColor;
public:
    void setup() override;
    void loop() override;
};


REGISTER_CONTROLLER(foraging_4_controller, "foraging_4_controller")
