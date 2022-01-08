#include "krembot.ino.h"



void foraging_4_controller::setup() {
    krembot.setup();
    writeTeamColor();
    teamName = "foraging_4_controller";
}

void foraging_4_controller::loop() {
    krembot.loop();

    switch (state) {
        case State::move: {
            Colors colorFront = krembot.RgbaFront.readColor();
            Colors colorFrontLeft = krembot.RgbaFrontLeft.readColor();
            Colors colorFrontRight = krembot.RgbaFrontRight.readColor();
            float distance = krembot.RgbaFront.readRGBA().Distance;
            if ((distance < 25) || (colorFront != Colors::None)  ||
                 (colorFrontLeft !=  Colors::None) || (colorFrontRight !=  Colors::None)) {
                sandTimer.start(200);
                state = State::turn;
//                direction = rand() % 2;
//                if(direction == 0)
//                {
//                    direction = -1;
//                }
            } else {
                krembot.Base.drive(100, 0);
            }
            break;
        }

        case State::turn: {
            if (sandTimer.finished()) {
                state = State::move;
            } else {
                krembot.Base.drive(0, direction * turning_speed);
            }
            break;
        }
    }

}

