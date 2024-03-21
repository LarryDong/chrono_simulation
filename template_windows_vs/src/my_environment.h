#pragma once

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualShape.h"

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

         //Flat terrain
        //patch = terrain_.AddPatch(patch_mat, CSYSNORM, 500, 500);  // Generate a "flat" terrain
        

        // obj terrain
        //patch = terrain_.AddPatch(patch_mat, ChCoordsys<>({ 0, 0, -15.0 }, Q_from_AngAxis(3.1415/2, chrono::Vector(1,0,0))), 
        //            "C:/Users/larrydong/Desktop/123.obj");
        
        // BMP terrain.
        double terrain_size = 200;
        std::string height_bmp = "C:/Users/larrydong/Desktop/step1.bmp";
        patch = terrain_.AddPatch(patch_mat, ChCoordsys<>({ 0, 0, -0.5 }, QUNIT), height_bmp, terrain_size, terrain_size, 0.0, 0.2);   // 64x64£¬´¿ºÚÏñËØ0£¬´¿°×1
        patch->SetTexture(GetDataFile("terrain/textures/grass.jpg"), terrain_size, terrain_size);
        patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
        terrain_.Initialize();
        std::cout << "--> terrain inited. " << std::endl;

        // create surrounding environment;
        std::string scene_3d = "C:/Users/larrydong/Desktop/demo2.stl";
        auto mmesh = chrono::geometry::ChTriangleMeshConnected::CreateFromSTLFile(scene_3d);
        double inch_2_mm_scale = 0.0254;
        mmesh->Transform(ChVector<>(0, 0, -0.5), ChMatrix33<>(1));  // scale from inch -> mm.
        //mmesh->Transform(ChVector<>(0, 0, -0.5), ChMatrix33<>(1.0 / (1.0f / 0.0254)));  // scale from inch -> mm.
        std::cout << "--> 3D scene inited. " << std::endl;

        auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(mmesh);
        trimesh_shape->SetName("HMMWV Chassis Mesh");
        trimesh_shape->SetMutable(false);

        auto mesh_body = chrono_types::make_shared<ChBody>();
        mesh_body->SetPos({ 0, 0, 0 });
        mesh_body->AddVisualShape(trimesh_shape, ChFrame<>());
        mesh_body->SetBodyFixed(true);
        sys->Add(mesh_body);
    }

public:
	chrono::vehicle::RigidTerrain terrain_;
};

