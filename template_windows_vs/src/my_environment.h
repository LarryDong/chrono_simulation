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
        //patch = terrain_.AddPatch(patch_mat, CSYSNORM, 100.0, 100.0);  // Generate a "flat" terrain
        // Generate a BMP terrain.
		ChCoordsys<> coordinate = ChCoordsys<>({ 0, 0, -0.5 }, QUNIT);
        std::string height_bmp = "C:/Users/larrydong/Desktop/my_terrain.bmp";
        patch = terrain_.AddPatch(patch_mat, coordinate, height_bmp, 100.0, 100.0, 0.0, 0.2);   // 64x64£¬´¿ºÚÏñËØ0£¬´¿°×1
        patch->SetTexture(GetDataFile("terrain/textures/grass.jpg"), 100, 100);
        patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));
        terrain_.Initialize();

        // create surrounding environment;        // TODO:

        std::string scene_3d = "C:/Users/larrydong/Desktop/test.stl";
        auto mmesh =chrono::geometry::ChTriangleMeshConnected::CreateFromSTLFile(scene_3d);
        mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(0.2));  // scale to a different size

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

