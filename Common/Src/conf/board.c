#include "conf/board.h"

// DeviceBoardConf_t* BoardConfGetDeviceById (eDEVICE_ID_t deviceId) {

//     BoardConf_t const* pBoardConf = BoardConfGet ();
//     for (uint32_t i = 0; i < pBoardConf->numDevices; ++i) {
//         if (pBoardConf->pDeviceBoardConfs[i].deviceId == deviceId) {
//             return &pBoardConf->pDeviceBoardConfs[i];
//         }
//     }
//     return 0;
// }

// ServoBoardConf_t* BoardConfGetServoById (eDEVICE_ID_t servoId) {

//     BoardConf_t const* pBoardConf = BoardConfGet ();
//     for (uint32_t i = 0; i < pBoardConf->numServos; ++i) {
//         if (pBoardConf->pServoBoardConfs[i].servoId == servoId) {
//             return &pBoardConf->pServoBoardConfs[i];
//         }
//     }
//     return 0;
// }

// MotorBoardConf_t* BoardConfGetMotorById (eDEVICE_ID_t motorId) {

//     BoardConf_t const* pBoardConf = BoardConfGet ();
//     for (uint32_t i = 0; i < pBoardConf->numMotors; ++i) {
//         if (pBoardConf->pMotorBoardConfs[i].motorId == motorId) {
//             return &pBoardConf->pMotorBoardConfs[i];
//         }
//     }
//     return 0;
// }