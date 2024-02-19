#pragma once

#include "chrono/physics/ChSystem.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"


class MyEnvironment {

public:
    
    MyEnvironment(chrono::ChSystem* sys) :terrain_(sys) {
        // create terrain;
        using namespace chrono;
        using namespace chrono::vehicle;
        ChContactMaterialData minfo;
        minfo.mu = 0.9f;
        minfo.cr = 0.01f;
        minfo.Y = 2e7f;
        auto patch_mat = minfo.CreateMaterial(ChContactMethod::NSC);


        std::shared_ptr<RigidTerrain::Patch> patch;
        // Generate a "flat" terrain
        //patch = terrain_.AddPatch(patch_mat, CSYSNORM, 100.0, 100.0);
        // Generate a BMP terrain.
        std::string height_bmp = "C:/Users/larrydong/Desktop/my_terrain.bmp";
        patch = terrain_.AddPatch(patch_mat, CSYSNORM, height_bmp, 100.0, 100.0, 0.0, 0.2);   // 64x64£¬´¿ºÚÏñËØ0£¬´¿°×1
        patch->SetTexture(GetDataFile("terrain/textures/grass.jpg"), 100, 100);
        patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
        terrain_.Initialize();

        // create surrounding environment;        // TODO:
    }

public:
	chrono::vehicle::RigidTerrain terrain_;
};

