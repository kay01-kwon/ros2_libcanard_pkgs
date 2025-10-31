#include "rc_converter.hpp"
#include "single_model.hpp"
#include "quad_model.hpp"
#include "hexa_model.hpp"

RCConverter::~RCConverter()
{

}

RCConverter* RCConverter::create_RCConverter(DroneParam drone_param)
{
    DroneModel model = drone_param.drone_model;

    switch (model)
    {
        case DroneModel::SINGLE:
            // Return an instance of SingleModelConverter
            return new SingleModelConverter();
        case DroneModel::QUAD:
            // Return an instance of QuadModelConverter
            return new QuadModelConverter(drone_param);
        case DroneModel::HEXA:
            // Return an instance of HexaModelConverter
            return new HexaModelConverter(drone_param);
        default:
            std::cerr << "Invalid RC Mode!" << std::endl;
            return nullptr;
    }
}