#ifndef DOBOTDLL_H
#define DOBOTDLL_H

#include "dobotdll_global.h"
#include "DobotType.h"

extern "C" DOBOTDLLSHARED_EXPORT int DobotExec(void);

extern "C" DOBOTDLLSHARED_EXPORT int SearchDobot(char *dobotNameList, uint32_t maxLen);
extern "C" DOBOTDLLSHARED_EXPORT int ConnectDobot(const char *portName, uint32_t baudrate);
extern "C" DOBOTDLLSHARED_EXPORT int DisconnectDobot(void);

extern "C" DOBOTDLLSHARED_EXPORT int SetCmdTimeout(uint32_t cmdTimeout);

// Device information
extern "C" DOBOTDLLSHARED_EXPORT int SetDeviceSN(const char *deviceSN);
extern "C" DOBOTDLLSHARED_EXPORT int GetDeviceSN(char *deviceSN, uint32_t maxLen);

extern "C" DOBOTDLLSHARED_EXPORT int SetDeviceName(const char *deviceName);
extern "C" DOBOTDLLSHARED_EXPORT int GetDeviceName(char *deviceName, uint32_t maxLen);

extern "C" DOBOTDLLSHARED_EXPORT int GetDeviceVersion(uint8_t *majorVersion, uint8_t *minorVersion, uint8_t *revision);

// Pose and Kinematics parameters are automatically get
extern "C" DOBOTDLLSHARED_EXPORT int GetPose(Pose *pose);
extern "C" DOBOTDLLSHARED_EXPORT int GetPoseEx(Pose *pose);
extern "C" DOBOTDLLSHARED_EXPORT int ResetPose(bool manual, float rearArmAngle, float frontArmAngle);
extern "C" DOBOTDLLSHARED_EXPORT int GetKinematics(Kinematics *kinematics);

// Alarms
extern "C" DOBOTDLLSHARED_EXPORT int GetAlarmsState(uint8_t *alarmsState, uint32_t *len, uint32_t maxLen);
extern "C" DOBOTDLLSHARED_EXPORT int ClearAllAlarmsState(void);

// HOME
extern "C" DOBOTDLLSHARED_EXPORT int SetHOMEParams(HOMEParams *homeParams, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" DOBOTDLLSHARED_EXPORT int GetHOMEParams(HOMEParams *homeParams);

extern "C" DOBOTDLLSHARED_EXPORT int SetHOMECmd(HOMECmd *homeCmd, bool isQueued, uint64_t *queuedCmdIndex);

// Handheld teach
extern "C" DOBOTDLLSHARED_EXPORT int SetHHTTrigMode(HHTTrigMode hhtTrigMode);
extern "C" DOBOTDLLSHARED_EXPORT int GetHHTTrigMode(HHTTrigMode *hhtTrigMode);

extern "C" DOBOTDLLSHARED_EXPORT int SetHHTTrigOutputEnabled(bool isEnabled);
extern "C" DOBOTDLLSHARED_EXPORT int GetHHTTrigOutputEnabled(bool *isEnabled);

extern "C" DOBOTDLLSHARED_EXPORT int GetHHTTrigOutput(bool *isTriggered);

// EndEffector
extern "C" DOBOTDLLSHARED_EXPORT int SetEndEffectorParams(EndEffectorParams *endEffectorParams, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" DOBOTDLLSHARED_EXPORT int GetEndEffectorParams(EndEffectorParams *endEffectorParams);

extern "C" DOBOTDLLSHARED_EXPORT int SetEndEffectorLaser(bool enableCtrl, bool on, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" DOBOTDLLSHARED_EXPORT int GetEndEffectorLaser(bool *isCtrlEnabled, bool *isOn);

extern "C" DOBOTDLLSHARED_EXPORT int SetEndEffectorSuctionCup(bool enableCtrl, bool suck, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" DOBOTDLLSHARED_EXPORT int GetEndEffectorSuctionCup(bool *isCtrlEnabled, bool *isSucked);

extern "C" DOBOTDLLSHARED_EXPORT int SetEndEffectorGripper(bool enableCtrl, bool grip, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" DOBOTDLLSHARED_EXPORT int GetEndEffectorGripper(bool *isCtrlEnabled, bool *isGripped);

// Arm orientation
extern "C" DOBOTDLLSHARED_EXPORT int SetArmOrientation(ArmOrientation armOrientation, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" DOBOTDLLSHARED_EXPORT int GetArmOrientation(ArmOrientation *armOrientation);

// JOG functions
extern "C" DOBOTDLLSHARED_EXPORT int SetJOGJointParams(JOGJointParams *jointJogParams, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" DOBOTDLLSHARED_EXPORT int GetJOGJointParams(JOGJointParams *jointJogParams);

extern "C" DOBOTDLLSHARED_EXPORT int SetJOGCoordinateParams(JOGCoordinateParams *coordinateJogParams, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" DOBOTDLLSHARED_EXPORT int GetJOGCoordinateParams(JOGCoordinateParams *coordinateJogParams);

extern "C" DOBOTDLLSHARED_EXPORT int SetJOGCommonParams(JOGCommonParams *jogCommonParams, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" DOBOTDLLSHARED_EXPORT int GetJOGCommonParams(JOGCommonParams *jogCommonParams);
extern "C" DOBOTDLLSHARED_EXPORT int SetJOGCmd(JOGCmd *jogCmd, bool isQueued, uint64_t *queuedCmdIndex);

// PTP functions
extern "C" DOBOTDLLSHARED_EXPORT int SetPTPJointParams(PTPJointParams *ptpJointParams, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" DOBOTDLLSHARED_EXPORT int GetPTPJointParams(PTPJointParams *ptpJointParams);
extern "C" DOBOTDLLSHARED_EXPORT int SetPTPCoordinateParams(PTPCoordinateParams *ptpCoordinateParams, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" DOBOTDLLSHARED_EXPORT int GetPTPCoordinateParams(PTPCoordinateParams *ptpCoordinateParams);
extern "C" DOBOTDLLSHARED_EXPORT int SetPTPJumpParams(PTPJumpParams *ptpJumpParams, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" DOBOTDLLSHARED_EXPORT int GetPTPJumpParams(PTPJumpParams *ptpJumpParams);
extern "C" DOBOTDLLSHARED_EXPORT int SetPTPCommonParams(PTPCommonParams *ptpCommonParams, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" DOBOTDLLSHARED_EXPORT int GetPTPCommonParams(PTPCommonParams *ptpCommonParams);

extern "C" DOBOTDLLSHARED_EXPORT int SetPTPCmd(PTPCmd *ptpCmd, bool isQueued, uint64_t *queuedCmdIndex);

// CP functions
extern "C" DOBOTDLLSHARED_EXPORT int SetCPParams(CPParams *cpParams, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" DOBOTDLLSHARED_EXPORT int GetCPParams(CPParams *cpParams);
extern "C" DOBOTDLLSHARED_EXPORT int SetCPCmd(CPCmd *cpCmd, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" DOBOTDLLSHARED_EXPORT int SetCPLECmd(CPCmd *cpCmd, bool isQueued, uint64_t *queuedCmdIndex);

// ARC
extern "C" DOBOTDLLSHARED_EXPORT int SetARCParams(ARCParams *arcParams, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" DOBOTDLLSHARED_EXPORT int GetARCParams(ARCParams *arcParams);
extern "C" DOBOTDLLSHARED_EXPORT int SetARCCmd(ARCCmd *arcCmd, bool isQueued, uint64_t *queuedCmdIndex);

// WAIT
extern "C" DOBOTDLLSHARED_EXPORT int SetWAITCmd(WAITCmd *waitCmd, bool isQueued, uint64_t *queuedCmdIndex);

// TRIG
extern "C" DOBOTDLLSHARED_EXPORT int SetTRIGCmd(TRIGCmd *trigCmd, bool isQueued, uint64_t *queuedCmdIndex);

// EIO
extern "C" DOBOTDLLSHARED_EXPORT int SetIOMultiplexing(IOMultiplexing *ioMultiplexing, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" DOBOTDLLSHARED_EXPORT int GetIOMultiplexing(IOMultiplexing *ioMultiplexing);

extern "C" DOBOTDLLSHARED_EXPORT int SetIODO(IODO *ioDO, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" DOBOTDLLSHARED_EXPORT int GetIODO(IODO *ioDO);

extern "C" DOBOTDLLSHARED_EXPORT int SetIOPWM(IOPWM *ioPWM, bool isQueued, uint64_t *queuedCmdIndex);
extern "C" DOBOTDLLSHARED_EXPORT int GetIOPWM(IOPWM *ioPWM);

extern "C" DOBOTDLLSHARED_EXPORT int GetIODI(IODI *ioDI);
extern "C" DOBOTDLLSHARED_EXPORT int GetIOADC(IOADC *ioADC);

extern "C" DOBOTDLLSHARED_EXPORT int SetEMotor(EMotor *eMotor, bool isQueued, uint64_t *queuedCmdIndex);

// CAL
extern "C" DOBOTDLLSHARED_EXPORT int SetAngleSensorStaticError(float rearArmAngleError, float frontArmAngleError);
extern "C" DOBOTDLLSHARED_EXPORT int GetAngleSensorStaticError(float *rearArmAngleError, float *frontArmAngleError);
extern "C" DOBOTDLLSHARED_EXPORT int SetAngleSensorCoef(float rearArmAngleCoef, float frontArmAngleCoef);
extern "C" DOBOTDLLSHARED_EXPORT int GetAngleSensorCoef(float *rearArmAngleCoef, float *frontArmAngleCoef);

extern "C" DOBOTDLLSHARED_EXPORT int SetBaseDecoderStaticError(float baseDecoderError);
extern "C" DOBOTDLLSHARED_EXPORT int GetBaseDecoderStaticError(float *baseDecoderError);

// WIFI
extern "C" DOBOTDLLSHARED_EXPORT int SetWIFIConfigMode(bool enable);
extern "C" DOBOTDLLSHARED_EXPORT int GetWIFIConfigMode(bool *isEnabled);
extern "C" DOBOTDLLSHARED_EXPORT int SetWIFISSID(const char *ssid);
extern "C" DOBOTDLLSHARED_EXPORT int GetWIFISSID(char *ssid, uint32_t maxLen);
extern "C" DOBOTDLLSHARED_EXPORT int SetWIFIPassword(const char *password);
extern "C" DOBOTDLLSHARED_EXPORT int GetWIFIPassword(char *password, uint32_t maxLen);
extern "C" DOBOTDLLSHARED_EXPORT int SetWIFIIPAddress(WIFIIPAddress *wifiIPAddress);
extern "C" DOBOTDLLSHARED_EXPORT int GetWIFIIPAddress(WIFIIPAddress *wifiIPAddress);
extern "C" DOBOTDLLSHARED_EXPORT int SetWIFINetmask(WIFINetmask *wifiNetmask);
extern "C" DOBOTDLLSHARED_EXPORT int GetWIFINetmask(WIFINetmask *wifiNetmask);
extern "C" DOBOTDLLSHARED_EXPORT int SetWIFIGateway(WIFIGateway *wifiGateway);
extern "C" DOBOTDLLSHARED_EXPORT int GetWIFIGateway(WIFIGateway *wifiGateway);
extern "C" DOBOTDLLSHARED_EXPORT int SetWIFIDNS(WIFIDNS *wifiDNS);
extern "C" DOBOTDLLSHARED_EXPORT int GetWIFIDNS(WIFIDNS *wifiDNS);
extern "C" DOBOTDLLSHARED_EXPORT int GetWIFIConnectStatus(bool *isConnected);

// TEST
extern "C" DOBOTDLLSHARED_EXPORT int GetUserParams(UserParams *userParams);
extern "C" DOBOTDLLSHARED_EXPORT int GetPTPTime(PTPCmd *ptpCmd, uint32_t *ptpTime);

// Queued command
extern "C" DOBOTDLLSHARED_EXPORT int SetQueuedCmdStartExec(void);
extern "C" DOBOTDLLSHARED_EXPORT int SetQueuedCmdStopExec(void);
extern "C" DOBOTDLLSHARED_EXPORT int SetQueuedCmdForceStopExec(void);
extern "C" DOBOTDLLSHARED_EXPORT int SetQueuedCmdStartDownload(uint32_t totalLoop, uint32_t linePerLoop);
extern "C" DOBOTDLLSHARED_EXPORT int SetQueuedCmdStopDownload(void);
extern "C" DOBOTDLLSHARED_EXPORT int SetQueuedCmdClear(void);
extern "C" DOBOTDLLSHARED_EXPORT int GetQueuedCmdCurrentIndex(uint64_t *queuedCmdCurrentIndex);

#endif // DOBOTDLL_H
