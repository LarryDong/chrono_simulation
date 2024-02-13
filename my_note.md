


## Issues
- what is `marker`??
- what is `quadrature`	


- 为什么lidar需要attach到一个parent上？
- Sensor的更新频率/运动控制，是如何在时间上连续的？


## Abbr.
`trsf`: transfer
`QUINT`: unit quatarion
`glyph`: 石雕符号；象形文字
`FEA`: 有限元分析
`COG`: center of mass
`NSC`: Non Smooth Contacts
`SMC`: SMooth Contacts 
`IRR`: irrlicht
`TSDA`: spring-damper-actuator





chrono_types	Namespace for custom make_shared implementation，前面不需要namespace，在core里面。
AddTypicalLights：没有光线，是黑的？？


## Reminder

**std::chrono confliction**  
If `using namespace std`,  `chrono` will be implict due to `std::chrono`

**wheel rotate speed**  
ChLinkMotorRotationSpeed

**ChMaterialSurfaceNSC**  
is derived from: `ChMaterialSurface`, which has: Rolling friction 滚动摩擦; sliding_friction; spinning_friction(?? What's this)

**Modify the default data path**  
Use function: "SetChronoDataPath("/home/larrydong/codeGit/chrono_simulation/data")"  
then `GetChronoDataFile` will load data file from the setting

**碰撞、可视化**
碰撞体积：ChMaterialSurfaceNSC  
可视化体积：ChVisualShapeBox

- chrono_types：Namespace for custom make_shared implementation，前面不需要namespace，在core里面。
- Visulization时需要添加光线，AddTypicalLight，否则场景的物体是黑色的，看不到。


