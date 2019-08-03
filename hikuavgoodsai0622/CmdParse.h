/**	@file       CmdParse.h
 *	@note       Hikvision Digital Technology Co., Ltd. All Right Reserved.
 *	@brief
 *
 *	@author     lipengfei
 *	@date       2018/05/10
 *	@note       历史记录：
 *	@note       V1.0.0
 *	@warning
 */
#ifndef __CMDPARSE_H__
#define __CMDPARSE_H__

#include "JsonParse.h"
#include <vector>

#define SOCKET_HEAD_LEN          8                      ///< 8个字节的头部长度


typedef struct _CONNECT_NOTICE_
{
    char    szNotice[64];
    char    szMsg[128];
}CONNECT_NOTICE;

typedef struct _TOKEN_INFO_
{
    char    szToken[64];
    char    szAction[64];
}TOKEN_INFO;


typedef struct _TOKEN_RESULT_
{
    char    szToken[64];
    char    szNotice[64];
    char    szRoundId[64];
    char    szPalyerId[64];
    int     nResult;
}TOKEN_RESULT;

typedef struct _READY_PARAM_
{
    char    szToken[64];
    char    szAction[64];
}READY_PARAM;



typedef struct _BUILDING_
{
    int     nX;
    int     nY;
    int     nL;
    int     nW;
    int     nH;
}BUILDING;

typedef struct _FOG_
{
    int     nX;
    int     nY;
    int     nL;
    int     nW;
    int     nB;
    int     nT;
}FOG;




#define MAX_BUILDING_NUM        128
#define MAX_FOG_NUM        128

#define MAX_UAV_NUM         512

#define MAX_UAV_PRICE_NUM    64

#define MAX_GOODS_NUM       256



typedef enum _UAV_STATUS_
{
    UAV_NOMAL = 0,
    UAV_CRASH,
    UAV_FOG
}UAV_STATUS;

typedef struct _UAV_
{
    int     nNO;//无人机编号
    char    szType[8];//无人机类型，跟价格对应
    int     nX;//无人机坐标
    int     nY;
    int     nZ;
    int     nLoadWeight;            ///< 跟type对应的无人机的载重一样，
    UAV_STATUS  nStatus;   ///无人机状态，正常，撞毁，大雾区
    int     nGoodsNo; ///商品编号，-1表示没有载货，
    int     nWorkStatus;///0自由,1取货,2 放货,3，攻击 4.待机 5.去充电
    int     nGoodNoTarget;///
    int     nEndPoint[6]; ///目标坐标
    std::vector <int> nWay;///路径坐标
    //int     PointNo;        ///锁定点序号

    int     nTargetEnemyUav;///己方飞机目标追击敌方飞机
    int     nWeUavWhetherChase;///己方飞机是否有在追击敌方飞机，0没有追击，1有追击
    int     nEnemyUavWhetherChase;///敌方飞机是否有被追击，0未被追击，1被追击
    int     nEnemyUavBeChasedNo;///敌方飞机被己方飞机追击己方飞机编号

    int     nRemainElectricity;///无人机剩余电量
    int     nLoadGoodsWeight;///载的货物重量

}UAV;


typedef struct _UAV_PRICE_
{
    char    szType[8];  //无人机类型
    int     nLoadWeight; //载重量
    int     nValue;  //价格

    int     nCapacity;  //总电量
    int     nCharge;  //单位时间充电

}UAV_PRICE;

/** @struct
 * 	@brief
 *	@note
 */
typedef struct _MAP_INFO_
{
    int     nMapX;//x，y，z
    int     nMapY;
    int     nMapZ;
    int     nParkingX; //停机坪
    int     nParkingY;

    int     nHLow; //飞行最低和 最高高度
    int     nHHigh;

    int     nBuildingNum; //建筑物数量
    BUILDING    astBuilding[MAX_BUILDING_NUM];  //建筑物坐标信息
    int     nFogNum; //大雾区，整个比赛中不变
    FOG         astFog[MAX_FOG_NUM];  //误区坐标信息
    int     nUavNum;//无人机数量
    UAV     astUav[MAX_UAV_NUM];//无人机信息
    int     nUavPriceNum;//无人机价格种类
    UAV_PRICE   astUavPrice[MAX_UAV_PRICE_NUM];//价格无人机种类信息
}MAP_INFO;


typedef struct _FLAY_PLANE_
{
    int     nUavNum;//飞机数量
    UAV     astUav[MAX_UAV_NUM];//信息

    int     nPurchaseNum;//购买无人机数量
    char    szPurchaseType[MAX_UAV_PRICE_NUM][8];//购买服务器类型
}FLAY_PLANE;

typedef struct _GOODS_
{
    int     nNO;//货物编号
    int     nStartX; //货物出现位置
    int     nStartY;
    int     nEndX;//获取目的位置
    int     nEndY;
    int     nWeight;//货物重量
    int     nValue;//货物价值
    int     nStartTime;//货物出现的时间
    int     nRemainTime;//货物持续时间
    int     nLeftTime;//剩余时间
    int     nState;//货物状态，0未拾取到．1表示拾取，2表示送到，3表示无效

    int     nLockStatus;//0自由，1 lock
}GOODS;

typedef struct _MATCH_STATUS_
{
    int     nTime;      //当前时间
    int     nMacthStatus;            //比赛状态
    int     nUavWeNum;      //我们的无人机数量
    UAV     astWeUav[MAX_UAV_NUM]; //我们无人机状态
    int     nWeValue;  //我方目前价值
    int     nUavEnemyNum;  //敌方无人机数量
    UAV     astEnemyUav[MAX_UAV_NUM]; //敌方无人机
    int     nEnemyValue;//敌方价值
    int     nGoodsNum;//实时货物信息
    GOODS   astGoods[MAX_GOODS_NUM];
}MATCH_STATUS;

typedef struct _WAIT_POINT_
{
    int WX;
    int WY;
    int WZ;
    int PointStatus;//0-空 1-已安排飞机
}Wait_Point;

/** @fn     int ParserConnect(char *pBuffer, CONNECT_NOTICE *pstNotice)
 *  @brief
 *	@param  -I   - char * pBuffer
 *	@param  -I   - CONNECT_NOTICE * pstNotice
 *	@return int
 */
int ParserConnect(char *pBuffer, CONNECT_NOTICE *pstNotice);


/** @fn     int ParserTokenResult(char *pBuffer, TOKEN_RESULT *pResult)
 *  @brief
 *	@param  -I   - char * pBuffer
 *	@param  -I   - TOKEN_RESULT * pResult
 *	@return int
 */
int ParserTokenResult(char *pBuffer, TOKEN_RESULT *pResult);


/** @fn     int ParserMapInfo(char *pBuffer, MAP_INFO *pstMap)
 *  @brief
 *	@param  -I   - char * pBuffer
 *	@param  -I   - MAP_INFO * pstMap
 *	@return int
 */
int ParserMapInfo(char *pBuffer, MAP_INFO *pstMap);


/** @fn     int ParserUav(cJSON *pUavArray, UAV *astUav, int *pNum)
 *  @brief
 *	@param  -I   - cJSON * pUavArray
 *	@param  -I   - UAV * astUav
 *	@param  -I   - int * pNum
 *	@return int
 */
int ParserUav(cJSON *pUavArray, UAV *astUav, int *pNum);

/** @fn     int ParserMatchStatus(char *pBuffer, MATCH_STATUS *pstStatus)
 *  @brief
 *	@param  -I   - char * pBuffer
 *	@param  -I   - MATCH_STATUS * pstStatus
 *	@return int
 */
int ParserMatchStatus(char *pBuffer, MATCH_STATUS *pstStatus);


/** @fn     int CreateTokenInfo(TOKEN_INFO *pstInfo, char *pBuffer)
 *  @brief
 *	@param  -I   - TOKEN_INFO * pstInfo
 *	@param  -I   - char * pBuffer
 *	@return int
 */
int CreateTokenInfo(TOKEN_INFO *pstInfo, char *pBuffer, int *pLen);

/** @fn     int CreateReadyParam(READY_PARAM *pstParam, char *pBuffer, int *pLen)
 *  @brief
 *	@param  -I   - READY_PARAM * pstParam
 *	@param  -I   - char * pBuffer
 *	@param  -I   - int * pLen
 *	@return int
 */
int CreateReadyParam(READY_PARAM *pstParam, char *pBuffer, int *pLen);


/** @fn     int CreateFlayPlane(FLAY_PLANE *pstPlane, char *pBuffer, int *pLen)
 *  @brief
 *	@param  -I   - FLAY_PLANE * pstPlane
 *	@param  -I   - char * pBuffer
 *	@param  -I   - int * pLen
 *	@return int
 */
int CreateFlayPlane(FLAY_PLANE *pstPlane, char *szToken, char *pBuffer, int *pLen);

#endif

