#include "rc_converter.hpp"
#include "single_mode.hpp"
RCConverter::~RCConverter()
{

}

RCConverter* RCConverter::create_RCConverter(RCMode mode)
{
    switch (mode)
    {
        case RCMode::SINGLE:
            // Return an instance of SingleModeConverter
            return new SingleModeConverter();
        case RCMode::QUAD:
            // Return an instance of QuadModeConverter
            // return new QuadModeConverter();
            std::cerr << "QuadModeConverter not implemented yet." << std::endl;
            return nullptr;
        case RCMode::HEXA:
            // Return an instance of HexaModeConverter
            // return new HexaModeConverter();
            std::cerr << "HexaModeConverter not implemented yet." << std::endl;
            return nullptr;
        default:
            std::cerr << "Invalid RC Mode!" << std::endl;
            return nullptr;
    }
}