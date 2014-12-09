#include "spacecraftcontroller.h"

SpacecraftController::SpacecraftController(const int &dimensions) : dimensions_(dimensions) {

}

SpacecraftController::~SpacecraftController()
{

}

int SpacecraftController::Dimensions() const
{
    return dimensions_;
}
