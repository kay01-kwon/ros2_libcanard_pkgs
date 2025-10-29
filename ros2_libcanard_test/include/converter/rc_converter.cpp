#include "rc_converter.hpp"
#include "single_mode.hpp"
#include "quad_mode.hpp"
#include "hexa_mode.hpp"

RCConverter::~RCConverter()
{

}

RCConverter* RCConverter::create_RCConverter(DroneParam drone_param)
{
    RCMode mode = drone_param.rc_mode;

    switch (mode)
    {
        case RCMode::SINGLE:
            // Return an instance of SingleModeConverter
            return new SingleModeConverter();
        case RCMode::QUAD:
            // Return an instance of QuadModeConverter
            return new QuadModeConverter(drone_param);
        case RCMode::HEXA:
            // Return an instance of HexaModeConverter
            return new HexaModeConverter(drone_param);
        default:
            std::cerr << "Invalid RC Mode!" << std::endl;
            return nullptr;
    }
}