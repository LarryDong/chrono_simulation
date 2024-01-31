


## Issues
- what is `marker`??
- what is `quadrature`



## Abbr.
`trsf`: transfer
`QUINT`: unit quatarion
`glyph`: 石雕符号；象形文字
`FEA`: 有限元分析




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

