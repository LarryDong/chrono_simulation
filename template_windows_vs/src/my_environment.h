#pragma once

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualShape.h"

#include "config.h"

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

        // 1. Flat terrain
        //patch = terrain_.AddPatch(patch_mat, CSYSNORM, 500, 500);  // Generate a "flat" 
        //patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
        
        // 2. BMP terrain.
        //double terrain_size = 200;
        //std::string height_bmp = "C:/Users/larrydong/Desktop/step1.bmp";
        //patch = terrain_.AddPatch(patch_mat, ChCoordsys<>({ 0, 0, -1}, QUNIT), height_bmp, terrain_size, terrain_size, 0.0, 1);   // 64x64，纯黑像素0，纯白
        //patch->SetTexture(GetDataFile("terrain/textures/grass.jpg"), terrain_size, terrain_size);
        
        // 3. OBJ terrain
        std::cout << "--> Loading terrain... " << std::endl;
        std::string terrain_obj = TERRAIN_OBJ;     // TODO: 注意单位（mm），以及z轴不要有遮挡。
        //std::string terrain_obj = "C:/Users/larrydong/Desktop/chrono_file/terrain/step_single_10cm.obj";     // TODO: 注意单位（mm），以及z轴不要有遮挡。
        patch = terrain_.AddPatch(patch_mat, ChCoordsys<>({ 0, 0, -0.5}, QUNIT), terrain_obj);
        std::cout << "<-- Terrain loaded. " << std::endl;
        terrain_.Initialize();


        std::string scene_3d = SCENE_OBJ;
        auto mmesh = chrono::geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(scene_3d, true, true);

        double inch_2_mm_scale = 0.0254;
        mmesh->Transform(ChVector<>(-0, -0, -0.5), ChMatrix33<>(0.01));
        auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(mmesh);
        trimesh_shape->SetName("scene mesh");
        trimesh_shape->SetMutable(false);
        auto mesh_body = chrono_types::make_shared<ChBody>();
        mesh_body->SetPos({ 0, 0, 0 });
        mesh_body->AddVisualShape(trimesh_shape, ChFrame<>());
        mesh_body->SetBodyFixed(true);
        std::cout << "--> Adding mesh. " << std::endl;
        sys->Add(mesh_body);
        std::cout << "<-- Mesh added. " << std::endl;
    }

public:
	chrono::vehicle::RigidTerrain terrain_;
};

