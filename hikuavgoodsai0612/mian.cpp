/**	@file       mian.cpp
 *	@note       Hikvision Digital Technology Co., Ltd. All Right Reserved.
 *	@brief
 *
 *	@author     lipengfei
 *	@date       2018/05/10
 *	@note       历史记录：
 *	@note       V1.0.0
 *	@warning
 */

#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include "OSSocket.h"
#include "JsonParse.h"
#include "CmdParse.h"
#include <queue>
#include <map>
#include <algorithm>
#include<time.h>
#include <iostream>
# include <stack>
using namespace std;
#define MAX_SOCKET_BUFFER       (1024 * 1024 * 4)       /// 发送接受数据最大4M
int ***space ;//= new int[200][200][200];//Z,X,Y

int enemyParkingX;//敌方停机坪
int enemyParkingY;
///自定义全局变量
#define INF                     0x3f3f3f3f
typedef pair<int, int> P;
struct point
{
    int x;
    int y;
    int z;
};
struct goodInfo
{
    int nLockStatus;//0自由，1 lock
    int distance;//取货放货最短距离
    int TarUav;//目标无人机
};

//表明每次x和y方向的位移 he 对角方向的位移（左上＼右上＼右下＼左下）he 竖直上＼下 he 不动
int dx[11] = { 1, 0, -1, 0, -1, -1, 1, 1, 0, 0, 0 }, dy[11] = { 0, 1, 0, -1, -1, 1, 1, -1, 0, 0, 0 }, dz[11] = { 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0 };
int **dis;
int **LastDis;
int  ***Dg;

vector<goodInfo> goodsInfo;//货物信息
vector<UAV_PRICE> uavTypeSort;//存放飞机种类排序后信息
int maxFogNum = 0;//找到面积最大的雾区
int enemyBeChased[512];//追踪敌方飞机标记

map<int, int> loadWeightCapacity; //存放飞机的载重和电容量
map<int, int> loadWeightCharge; //存放飞机的载重和单位时间充电量

/** @fn     int RecvJuderData(OS_SOCKET hSocket, char *pBuffer)
 *  @brief	接受数据
 *	@param  -I   - OS_SOCKET hSocket
 *	@param  -I   - char * pBuffer
 *	@return int
 */
int RecvJuderData(OS_SOCKET hSocket, char *pBuffer)
{
    int nRecvLen = 0;
    int nLen = 0;
    while(1)
    {
        // 接受头部长度
        nLen = OSRecv(hSocket, pBuffer + nRecvLen, MAX_SOCKET_BUFFER);
        if(nLen <= 0)
        {
            printf("recv error\n");
            return nLen;
        }
        nRecvLen += nLen;
        if(nRecvLen >= SOCKET_HEAD_LEN)
        {
            break;
        }
    }
    int nJsonLen = 0;
    char szLen[10] = { 0 };
    memcpy(szLen, pBuffer, SOCKET_HEAD_LEN);
    nJsonLen = atoi(szLen);
    while(nRecvLen < (SOCKET_HEAD_LEN + nJsonLen))
    {
        // 说明数据还没接受完
        nLen = OSRecv(hSocket, pBuffer + nRecvLen, MAX_SOCKET_BUFFER);
        if(nLen <= 0)
        {
            printf("recv error\n");
            return nLen;
        }
        nRecvLen += nLen;
    }
    return 0;
}

/** @fn     int SendJuderData(OS_SOCKET hSocket, char *pBuffer, int nLen)
 *  @brief	发送数据
 *	@param  -I   - OS_SOCKET hSocket
 *	@param  -I   - char * pBuffer
 *	@param  -I   - int nLen
 *	@return int
 */
int SendJuderData(OS_SOCKET hSocket, char *pBuffer, int nLen)
{
    int nSendLen = 0;
    int nLenTmp = 0;
    while(nSendLen < nLen)
    {
        nLenTmp = OSSend(hSocket, pBuffer + nSendLen, nLen - nSendLen);
        if(nLenTmp < 0)
        {
            return -1;
        }
        nSendLen += nLenTmp;
    }
    return 0;
}


//货物距离计算
int GoodDis(GOODS good, MAP_INFO *pstMap)
{
    point StaPoint = {good.nStartX, good.nStartY, pstMap->nHLow}; //货物起点
    point gpoint = {good.nEndX, good.nEndY, pstMap->nHLow}; //货物终点
    int Z = pstMap->nMapZ, X = pstMap->nMapX, Y = pstMap->nMapY;
    for(int z = 0; z < Z; z++) //
        for(int x = 0; x < X; x++)
            for(int y = 0; y < Y; y++)
                Dg[z][x][y] = INF;  //初始化所有点的距离为INF
    Dg[gpoint.z][gpoint.x][gpoint.y] = 0;  //从起点出发将距离设为0，并放入队列首端
    queue<point> que;
    que.push(gpoint);
    int distance = 0;
    while(que.size())  //题目保证有路到终点，所以不用担心死循环
    {
        point p = que.front();
        que.pop();//弹出队首元素
        int i;
        for(i = 0; i < 10; i++)
        {
            int nx = p.x  + dx[i];
            int ny = p.y  + dy[i];
            int nz = p.z  + dz[i];//移动后的坐标
            //判断可移动且没到过
            if(0 <= nx && nx < pstMap->nMapX
                    && 0 <= ny && ny < pstMap->nMapY
                    && pstMap->nHLow <= nz && nz < pstMap->nHHigh
                    && space[nz][nx][ny] != 1
                    && Dg[nz][nx][ny] == INF) //之前到过的话不用考虑，因为距离在队列中递增，肯定不会获得更好的解
            {
                que.push({nx, ny, nz});
                Dg[nz][nx][ny] = Dg[p.z][p.x][p.y] + 1;
                if(nx == StaPoint.x && ny == StaPoint.y && nz == StaPoint.z)
                {
                    distance = Dg[nz][nx][ny];
                    break;
                }
            }
        }
        if(i != 10)
            break;
    }
    distance = distance + 2 * pstMap->nHLow;
    return distance;
}

//以飞机为中心
int UavGood(UAV* Uav, MATCH_STATUS * pstMatch, FLAY_PLANE *pstFlayPlane, MAP_INFO *pstMap)
{
    vector<point> gpoint;//目的地
    //先找可以装载的货物的编号
    vector<int> canload;
    GOODS good ;
    for(int i = 0; i < pstMatch->nGoodsNum; i++)
    {
        good = pstMatch->astGoods[i];
        if(good.nState == 0 && Uav->nLoadWeight >= good.nWeight) //货物自由&&拿得起
        {
            if(goodsInfo[good.nNO].nLockStatus == 0) //未被锁定????
            {
                int len;
                if(abs(good.nStartX - Uav->nX) > abs(good.nStartY - Uav->nY))
                {
                    len = abs(good.nStartX - Uav->nX) + Uav->nZ;
                }
                else
                {
                    len = abs(good.nStartY - Uav->nY) + Uav->nZ;
                }
                if(len < good.nLeftTime)
                {
                    canload.push_back(i);
                    gpoint.push_back({good.nStartX, good.nStartY, Uav->nZ}); //目的地
                }
            }
        }
    }
    int num = canload.size();
    if(num == 0)
        return -1;//无解
    int Z = pstMap->nMapZ, X = pstMap->nMapX, Y = pstMap->nMapY;
    int *** d ;                                                          //创建三维数组
    d = new int**[Z] ;                                     //背包所装载的价值
    for(int i = 0 ; i < Z ; i ++)
    {
        d[i] = new int*[X] ;
        for(int j = 0 ; j < X; j ++)
        {
            d[i][j] = new int[Y] ;
            for(int k = 0 ; k < Y; k ++)
            {
                d[i][j][k] = INF ;
            }
        }
    }
    queue<point> que;
    que.push({Uav->nX, Uav->nY, Uav->nZ}); //初始点为 飞机地址，高度为最小Hlow
    d[Uav->nZ][Uav->nX][Uav->nY] = 0;  //从起点出发将距离设为0，并放入队列首端
    int allnum = num; //所有货物访问次数
    int real = num; //满足要求的货物点数......................可用更改
    int luckynum = 0; //存下来货物的号码
    int luckylen = 0;
    //性价比比较
    double xjb = 0; //性价比
    double temp = 0;
    int Len = INF;
    while(que.size() && real && allnum) //题目保证有路到终点，所以不用担心死循环
    {
        point p = que.front();
        que.pop();//弹出队首元素
        int i;
        for(i = 0; i < 10; i++)
        {
            int nx = p.x  + dx[i];
            int ny = p.y  + dy[i];
            int nz = p.z  + dz[i];//移动后的坐标
            //判断可移动且没到过
            if(0 <= nx && nx < pstMap->nMapX
                    && 0 <= ny && ny < pstMap->nMapY
                    && Uav->nZ <= nz && nz < pstMap->nHHigh
                    && space[nz][nx][ny] != 1
                    && d[nz][nx][ny] == INF) //之前到过的话不用考虑，因为距离在队列中递增，肯定不会获得更好的解
            {
                que.push({nx, ny, nz});
                d[nz][nx][ny] = d[p.z][p.x][p.y] + 1;
                Len = d[nz][nx][ny]; //总步长
                for(int n = 0; n < num; n++)
                {
                    if(nx == gpoint[n].x && ny == gpoint[n].y && nz == gpoint[n].z)
                    {
                        int gooddis = goodsInfo[pstMatch->astGoods[canload[n]].nNO].distance;
                        if(gooddis == 0)
                        {
                            gooddis = GoodDis(pstMatch->astGoods[canload[n]], pstMap);
                            goodsInfo[pstMatch->astGoods[canload[n]].nNO].distance = gooddis; //
                        }
                        if((Len + Uav->nZ) <= pstMatch->astGoods[canload[n]].nLeftTime
                                && Uav->nRemainElectricity > gooddis * pstMatch->astGoods[canload[n]].nWeight) //如果时间来得及&&电量足够
                        {
                            real--;//接近满足要求的数量
                            if(pstMatch->nTime < 60)
                            {
                                temp = (double)pstMatch->astGoods[canload[n]].nValue / gooddis; //性价比
                            }
                            else
                            {
                                temp = (double)pstMatch->astGoods[canload[n]].nValue / (Len + Uav->nZ * 2 + gooddis); //性价比
                            }
                            if(temp >= xjb)
                            {
                                luckynum = canload[n]; //编号
                                luckylen = Len; //距离
                                xjb = temp;
                            }
                        }
                        allnum--;//无论时间满足与否，总数减一
                    }
                }
            }
        }
    }
    if(real == num) //判断如果无解
        return -1;
    //更新飞机状态
    Uav->nWorkStatus = 1;                        //更新飞机工作状态为取货
    Uav->nGoodNoTarget = pstMatch->astGoods[luckynum].nNO;               //更新无人机目标物品编号
    Uav->nLoadGoodsWeight = pstMatch->astGoods[luckynum].nWeight;
    Uav->PointNo = -1;
    Uav->nEndPoint[0] = pstMatch->astGoods[luckynum].nStartX;             //更新目的地坐标
    Uav->nEndPoint[1] = pstMatch->astGoods[luckynum].nStartY;
    Uav->nEndPoint[2] = 0;
    Uav->nEndPoint[3] = pstMatch->astGoods[luckynum].nEndX;
    Uav->nEndPoint[4] = pstMatch->astGoods[luckynum].nEndY;
    Uav->nEndPoint[5] = 0;
    vector<int>().swap(Uav->nWay);
    //Point[Uav->PointNo].PointStatus=0;           //预置停机点放空
    goodsInfo[pstMatch->astGoods[luckynum].nNO].nLockStatus = 1;//锁定货物
    goodsInfo[pstMatch->astGoods[luckynum].nNO].TarUav = Uav->nNO;//锁定飞机
    //反推路径 栈
    stack<point> stk;
    stk.push({Uav->nEndPoint[0], Uav->nEndPoint[1], Uav->nZ});//压入货物
    for(int k = luckylen - 1; k >= 0; k--) //k=0是飞机点本身
    {
        point p = stk.top();//取出队首元素但不弹出
        for(int i = 0; i < 10; i++)
        {
            int nx = p.x + dx[i];
            int ny = p.y + dy[i];
            int nz = p.z + dz[i];
            if(0 <= nx && nx < pstMap->nMapX
                    && 0 <= ny && ny < pstMap->nMapY
                    && Uav->nZ <= nz && nz < pstMap->nHHigh
                    && d[nz][nx][ny] == k)//点存在并且距离满足
            {
                stk.push({nx, ny, nz});    //放入栈
                break;
            }
        }
    }
    stk.pop();//弹出末端（飞机起始点）
    for(int i = 0; i < luckylen; i++)
    {
        point p = stk.top();
        stk.pop();//弹出队首元素
        //压入！
        Uav->nWay.push_back(p.x);
        Uav->nWay.push_back(p.y);
        Uav->nWay.push_back(p.z);
    }
    //续一段
    for(int z = Uav->nZ - 1; z >= 0; z--)
    {
        Uav->nWay.push_back(pstMatch->astGoods[luckynum].nStartX);
        Uav->nWay.push_back(pstMatch->astGoods[luckynum].nStartY);
        Uav->nWay.push_back(z);
    }
    vector<int>().swap(canload);
    vector<point>().swap(gpoint);
    return (luckylen + Uav->nZ); //返回步长
}




//路径规划
int Waybfs(UAV* Uav, MAP_INFO *pstMap, FLAY_PLANE *pstFlayPlane, MATCH_STATUS * pstMatch) //vector<point>
{
    /*返回值-1:无人机坠亡
        返回值０:无解
        返回值>0:无人机有解
    */
    int len = INF; //总步长
    if(Uav->nStatus == 1)
        return len = -1;
    //
    if(Uav->nWay.size() > 0
            && Uav->nWay[0] == Uav->nEndPoint[0]
            && Uav->nWay[1] == Uav->nEndPoint[1]
            && Uav->nWay[2] == Uav->nEndPoint[2]
            && space[Uav->nWay[2]][Uav->nWay[0]][Uav->nWay[1]] == 3)
    {
        return len = 0;
    }
    if(Uav->nWay.size() == 3
            && Uav->nWay[0] == pstMap->nParkingX
            && Uav->nWay[1] == pstMap->nParkingY
            && Uav->nWay[2] == 0)
    {
        return 1;
    }
    //防碰撞机制
    //更改space,增加障碍物
    int cani = 1; //表示下一步能走
    vector<point> keepspace;//保留space现场位置
    vector<int> oldp;//保留现场的值
    if(Uav->nWay.size() > 0) //有规划路径
    {
        vector<point> enmy;//敌机所在位置
        vector<point> brother;//兄弟飞机
        point nowp = {Uav->nX, Uav->nY, Uav->nZ}; //当前点
        point next = {Uav->nWay[0], Uav->nWay[1], Uav->nWay[2]}; //下一步坐标
        bool Notime = false; //送货飞机时间不够了
        if(Uav->nWorkStatus == 2) //我方飞机送货但是电量不足
            Notime = true;
        //查找周围的敌机
        for(int i = 0; i < 11; i++)
        {
            int nx = next.x  + dx[i];
            int ny = next.y  + dy[i];
            int nz = next.z  + dz[i];//移动后的坐标
            if(0 <= nx && nx < pstMap->nMapX
                    && 0 <= ny && ny < pstMap->nMapY
                    && 0 <= nz && nz < pstMap->nHHigh)
            {
                if(nz < pstMap->nHLow && (dx[i] != 0 || dy[i] != 0)) //如果在nHlow一下而x\y移动
                {
                    continue;
                }
                if(space[nz][nx][ny] == 3)//自己人飞机
                {
                    brother.push_back({nx, ny, nz});
                }
                if(space[nz][nx][ny] == 4)//敌人飞机&&我方不是追击型
                {
                    if(Notime || Uav->nWorkStatus == 3)
                        continue;
                    enmy.push_back({nx, ny, nz});
                }
            }
        }
        //如果敌机存在，则下一步不能行走
        if(enmy.size() > 0)
        {
            cani = 0;   //下一步确定不能走
            for(int i = 0; i < 10; i++)
            {
                int nx = nowp.x  + dx[i];
                int ny = nowp.y  + dy[i];
                int nz = nowp.z  + dz[i];//移动后的坐标
                if(0 <= nx && nx < pstMap->nMapX
                        && 0 <= ny && ny < pstMap->nMapY
                        && 0 <= nz && nz < pstMap->nHHigh)
                {
                    if(nz < pstMap->nHLow && (dx[i] != 0 || dy[i] != 0)) //如果在nHlow一下而x\y移动
                        continue;
                    for(int j = 0; j < enmy.size(); j++)
                    {
                        for(int k = 0; k < 11; k++) //包括敌机自己
                        {
                            int x = enmy[j].x  + dx[k];
                            int y = enmy[j].y  + dy[k];
                            int z = enmy[j].z  + dz[k];//移动后的坐标
                            if(0 <= x && x < pstMap->nMapX
                                    && 0 <= y && y < pstMap->nMapY
                                    && 0 <= z && z < pstMap->nHHigh)
                            {
                                if(z < pstMap->nHLow && (dx[k] != 0 || dy[k] != 0)) //如果在nHlow一下而x\y移动
                                    continue;
                                if(nx == x && ny == y && nz == z) //如果有重叠区域
                                {
                                    keepspace.push_back({nx, ny, nz}); //将敌方飞机点变成建筑物１
                                    oldp.push_back(space[nz][nx][ny]);
                                    space[nz][nx][ny] = 1;
                                }
                            }
                        }
                    }
                }
            }
        }
        int  thisUavn = Uav->nNO;//本飞机序号
        if(brother.size() > 0) //
        {
            //找到周围兄弟飞机
            vector<int> Nb;
            for(int i = 0; i < brother.size(); i++)
            {
                for(int j = 0; j < pstFlayPlane->nUavNum; j++)
                {
                    if(pstFlayPlane->astUav[j].nStatus != 1
                            && brother[i].x == pstFlayPlane->astUav[j].nX
                            && brother[i].y == pstFlayPlane->astUav[j].nY
                            && brother[i].z == pstFlayPlane->astUav[j].nZ) //没死
                    {
                        Nb.push_back(j);//保存飞机号码
                    }
                }
            }
            if(cani == 1)                                                                       //如果没有敌机,检查自己的飞机会不会影响
            {
                if(space[next.z][next.x][next.y] == 3)
                    cani = 0;                                                           ///如果下一步被自己的飞机挡住
                for(int a = 0; a < Nb.size(); a++)
                {
                    UAV tempuav = pstFlayPlane->astUav[Nb[a]];
                    if(thisUavn > Nb[a])          ///或者前辈走过交叉
                    {
                        if(tempuav.nX + pstMatch->astWeUav[Nb[a]].nX == nowp.x + next.x
                                && tempuav.nY + pstMatch->astWeUav[Nb[a]].nY == nowp.y + next.y
                                && tempuav.nZ + pstMatch->astWeUav[Nb[a]].nZ == nowp.z + next.z)
                        {
                            cani = 0;
                        }
                    }
                }
            }
        }
        if(cani == 0) //如果下一步确定不能走
        {
            //将下一步设置为建筑物
            keepspace.push_back({next.x, next.y, next.z});
            oldp.push_back(space[next.z][next.x][next.y]);
            space[next.z][next.x][next.y] = 1;
            //求本机周边的兄弟飞机
            vector<point> Nowbrother;//兄弟飞机
            for(int i = 0; i < 10; i++)
            {
                int nx = nowp.x  + dx[i];
                int ny = nowp.y  + dy[i];
                int nz = nowp.z  + dz[i];//移动后的坐标
                if(0 <= nx && nx < pstMap->nMapX
                        && 0 <= ny && ny < pstMap->nMapY
                        && 0 <= nz && nz < pstMap->nHHigh)
                {
                    if(nz < pstMap->nHLow && (dx[i] != 0 || dy[i] != 0)) //如果在nHlow一下而x\y移动
                    {
                        continue;
                    }
                    if(space[nz][nx][ny] == 3)//自己人飞机
                    {
                        Nowbrother.push_back({nx, ny, nz});
                        keepspace.push_back({nx, ny, nz}); //将变成建筑物１
                        oldp.push_back(space[nz][nx][ny]);
                        space[nz][nx][ny] = 1;
                    }
                }
            }
            //周边有兄弟机，求前辈
            if(Nowbrother.size() > 0)
            {
                vector<int> NB;
                for(int i = 0; i < Nowbrother.size(); i++)
                {
                    for(int j = 0; j < pstFlayPlane->nUavNum; j++)
                    {
                        if(pstFlayPlane->astUav[j].nStatus != 1
                                && Nowbrother[i].x == pstFlayPlane->astUav[j].nX
                                && Nowbrother[i].y == pstFlayPlane->astUav[j].nY
                                && Nowbrother[i].z == pstFlayPlane->astUav[j].nZ) //没死
                        {
                            if(thisUavn > j)
                                NB.push_back(j);//保存前辈飞机号码
                        }
                    }
                }
                //如果有前辈机,看有没有交叉情况
                for(int a = 0; a < NB.size(); a++)
                {
                    UAV uav = pstFlayPlane->astUav[NB[a]];
                    for(int i = 0; i < 10; i++)
                    {
                        int nx = nowp.x  + dx[i];
                        int ny = nowp.y  + dy[i];
                        int nz = nowp.z  + dz[i];//移动后的坐标
                        if(0 <= nx && nx < pstMap->nMapX
                                && 0 <= ny && ny < pstMap->nMapY
                                && 0 <= nz && nz < pstMap->nHHigh)
                        {
                            if(nz < pstMap->nHLow && (dx[i] != 0 || dy[i] != 0)) //如果在nHlow一下而x\y移动
                            {
                                continue;
                            }
                            if(uav.nX + pstMatch->astWeUav[NB[a]].nX == nowp.x + nx
                                    && uav.nY + pstMatch->astWeUav[NB[a]].nY == nowp.y + ny
                                    && uav.nZ + pstMatch->astWeUav[NB[a]].nZ == nowp.z + nz)
                            {
                                Nowbrother.push_back({nx, ny, nz});
                                keepspace.push_back({nx, ny, nz}); //将变成建筑物１
                                oldp.push_back(space[nz][nx][ny]);
                                space[nz][nx][ny] = 1;
                            }
                        }
                    }
                }
            }
        }
        if(cani == 1)
            return 1;//无障碍，按照之前路径行走
    }
    int Z = pstMap->nMapZ, X = pstMap->nMapX, Y = pstMap->nMapY;
    /*
    for (int z = 0; z < Z; z++)//
        for (int x = 0; x < X; x++)
            for(int y = 0; y < Y; y++)
                d[z][x][y] = INF;  //初始化所有点的距离为INF
                */
    int *** d ;                                              //创建三维数组
    d = new int**[Z] ;                                      //背包所装载的价值
    for(int i = 0 ; i < Z ; i ++)
    {
        d[i] = new int*[X] ;
        for(int j = 0 ; j < X; j ++)
        {
            d[i][j] = new int[Y] ;
            for(int k = 0 ; k < Y; k ++)
            {
                d[i][j][k] = INF ;
            }
        }
    }
    point StaPoint = {Uav->nX, Uav->nY, Uav->nZ}; //飞机起点
    point gpoint = {Uav->nEndPoint[0], Uav->nEndPoint[1], Uav->nEndPoint[2]}; //终点
    d[gpoint.z][gpoint.x][gpoint.y] = 0;  //从起点出发将距离设为0，并放入队列首端
    queue<point> que;
    que.push(gpoint);
    while(que.size())  //题目保证有路到终点，所以不用担心死循环
    {
        point p = que.front();
        que.pop();//弹出队首元素
        int i;
        for(i = 0; i < 10; i++)
        {
            int nx = p.x  + dx[i];
            int ny = p.y  + dy[i];
            int nz = p.z  + dz[i];//移动后的坐标
            //判断可移动且没到过
            if(0 <= nx && nx < pstMap->nMapX
                    && 0 <= ny && ny < pstMap->nMapY
                    && 0 <= nz && nz < pstMap->nHHigh
                    && space[nz][nx][ny] != 1
                    && d[nz][nx][ny] == INF) //之前到过的话不用考虑，因为距离在队列中递增，肯定不会获得更好的解
            {
                if(nz < pstMap->nHLow && (dx[i] != 0 || dy[i] != 0)) //如果在nHlow一下而x\y移动
                    continue;
                que.push({nx, ny, nz});
                d[nz][nx][ny] = d[p.z][p.x][p.y] + 1;
                if(nx == StaPoint.x && ny == StaPoint.y && nz == StaPoint.z)
                    break;
            }
        }
        if(que.size() == 0) //判断如果无解
        {
            //让飞机不动
            //有障碍还原现场
            if(cani == 0)
            {
                for(int i = oldp.size() - 1; i >= 0; i--) //从后往前返回覆盖
                    space[keepspace[i].z][keepspace[i].x][keepspace[i].y] = oldp[i];
            }
            return len = 0; //无解
        }
        if(i != 10)
            break;
    }
    vector<int>().swap(Uav->nWay);
    queue<point> path;
    path.push(StaPoint);
    int nx = 0;
    int ny = 0;
    int nz = 0;//移动后的坐标
    len = d[StaPoint.z][StaPoint.x][StaPoint.y];
    //反推路径队列
    for(int k = len - 1; k >= 0; k--)
    {
        point p = path.back();//取出队首元素但不弹出
        for(int i = 0; i < 10; i++)
        {
            nx = p.x + dx[i];
            ny = p.y + dy[i];
            nz = p.z + dz[i];//移动后的坐标
            //判断可移动且没到过
            if(0 <= nx && nx < pstMap->nMapX
                    && 0 <= ny && ny < pstMap->nMapY
                    && 0 <= nz && nz < pstMap->nHHigh
                    && d[nz][nx][ny] == k) //点存在并且距离满足
            {
                if(nz < pstMap->nHLow && (dx[i] != 0 || dy[i] != 0)) //如果在nHlow一下而x\y移动
                    continue;
                path.push({nx, ny, nz});    //放入队列
                //压入！
                Uav->nWay.push_back(nx);
                Uav->nWay.push_back(ny);
                Uav->nWay.push_back(nz);
                break;
            }
        }
    }
    path.pop();//弹出第一个（起始点）
    //删除三维数组d
    for(int i = 0; i < Z; i++)
    {
        for(int j = 0; j < X; j++)
        {
            delete[] d[i][j];
        }
        delete[] d[i];
    }
    delete[] d;
    d = NULL;
    //有障碍还原现场
    if(cani == 0)
    {
        for(int i = oldp.size() - 1; i >= 0; i--) //从后往前返回覆盖
            space[keepspace[i].z][keepspace[i].x][keepspace[i].y] = oldp[i];
    }
    vector<point>().swap(keepspace);//保留space现场位置
    vector<int>().swap(oldp);//保留现场的值
    return len;
}

//对飞机种类按价值排序
int cmpUavType(UAV_PRICE uav1, UAV_PRICE uav2)
{
    if(uav1.nValue > uav2.nValue)
        return 0;
    else
        return 1;
}

/** @fn     void AlgorithmCalculationFun()
 *  @brief	学生的算法计算， 参数什么的都自己写，
 *	@return void
 */
void  AlgorithmCalculationFun(MAP_INFO *pstMap, MATCH_STATUS * pstMatch, FLAY_PLANE *pstFlayPlane)
{
    int MapX = pstMap->nMapX;
    int MapY = pstMap->nMapY;
    int MapZ = pstMap->nMapZ;
    //第一次更新一次就行
    if(pstMatch->nTime < 2)
    {
        //标记雾区
        int maxFogArea = -1;
        for(int i = 0; i < pstMap->nFogNum; i++)
        {
            int FX = pstMap->astFog[i].nX;//起始位置
            int FY = pstMap->astFog[i].nY;
            int FL = pstMap->astFog[i].nL;//雾区长度
            int FW = pstMap->astFog[i].nW;//雾区宽度
            int FB = pstMap->astFog[i].nB;//雾区最低高度
            int FT = pstMap->astFog[i].nT;//雾区最高高度
            for(int l = FX; l < (FX + FL); l++)
                for(int m = FY; m < (FY + FW); m++)
                    for(int n = FB; n < FT; n++)
                        space[n][l][m] = 2;//ChangeSpace(space,l,m,n,pstMap,2);
            //获得雾区序号
            if(FB <= pstMap->nHLow
                    && FT > (pstMap->nHLow + 1)
                    && FL * FW > maxFogArea)
            {
                maxFogArea = FL * FW;
                maxFogNum = i;
            }
        }
//        int minFogArea = 9999999;
//        //获取最小面积
//        for(int i = 0; i < fogNumTemp.size(); i++)
//        {
//            if(pstMap->astFog[fogNumTemp[i]].nL * pstMap->astFog[fogNumTemp[i]].nW < minFogArea)
//                minFogArea = pstMap->astFog[fogNumTemp[i]].nL * pstMap->astFog[fogNumTemp[i]].nW;
//        }
//        //获得雾区序列
//        for(int i = 0; i < fogNumTemp.size(); i++)
//        {
//            int ratioOfSize = pstMap->astFog[fogNumTemp[i]].nL * pstMap->astFog[fogNumTemp[i]].nW / minFogArea;
//            while(ratioOfSize--)
//            {
//                fogNum.push_back(fogNumTemp[i]);
//            }
//        }
        //标记UAV种类
        for(int i = 0; i < pstMap->nUavPriceNum; i++)
        {
            uavTypeSort.push_back(pstMap->astUavPrice[i]);
        }
        if(uavTypeSort.size() > 1)
        {
            sort(uavTypeSort.begin(), uavTypeSort.end(), cmpUavType);
        }
        //标记建筑物
        for(int i = 0; i < pstMap->nBuildingNum; i++)
        {
            int BX = pstMap->astBuilding[i].nX;
            int BY = pstMap->astBuilding[i].nY;
            int BL = pstMap->astBuilding[i].nL;
            int BW = pstMap->astBuilding[i].nW;
            int BH = pstMap->astBuilding[i].nH;
            //将不能飞行的区域标记为1
            for(int l = BX; l < (BL + BX); l++)
                for(int m = BY; m < (BW + BY); m++)
                    for(int n = 0; n < BH; n++)
                        space[n][l][m] = 1;//   ChangeSpace(space,l,m,n,pstMap,1);;
        }
        //清空飞机飞行信息
        for(int i = 0; i < pstMatch->nUavWeNum; i++)
        {
            vector<int>().swap(pstFlayPlane->astUav[i].nWay);
        }
        enemyParkingX = pstMatch->astEnemyUav[0].nX;
        enemyParkingY = pstMatch->astEnemyUav[0].nY;
        for(int i = 0; i < 256; i++)
        {
            goodsInfo.push_back({0, 0, 0});
        }
    }
    //更新飞机状态信息
    for(int i = 0; i < pstFlayPlane->nUavNum; i++)
    {
        pstFlayPlane->astUav[i].nStatus = pstMatch->astWeUav[i].nStatus;
    }
    //更新飞机数量信息
    for(int i = pstFlayPlane->nUavNum; i < pstMatch->nUavWeNum; i++)
    {
        pstFlayPlane->astUav[i] = pstMatch->astWeUav[i];
        pstFlayPlane->nUavNum++;
    }
    //清空购买信息
    for(int i = 0; i < pstFlayPlane->nPurchaseNum; i++)
    {
        for(int j = 0; j < 8; j++)
        {
            pstFlayPlane->szPurchaseType[i][j] = ' ';
        }
    }
    pstFlayPlane->nPurchaseNum = 0;
    //标记己方飞机
    for(int i = 0; i < pstMatch->nUavWeNum; i++)
    {
        int PX = pstMatch->astWeUav[i].nX;
        int PY = pstMatch->astWeUav[i].nY;
        int PZ = pstMatch->astWeUav[i].nZ;
        if(pstMatch->astWeUav[i].nStatus == 1) ///表示飞机坠毁，更新维0
        {
            space[PZ][PX][PY] = 0;
            continue;
        }
        space[PZ][PX][PY] = 3;//ChangeSpace(space,PX,PY,PZ,pstMap,3);
    }
    space[0][pstMap->nParkingX][pstMap->nParkingY] = 0;
    //自由状态达到待机状态更改
    int countWeSurvive = 0;
    for(int i = 0; i < pstFlayPlane->nUavNum; i++)
    {
        if(pstFlayPlane->astUav[i].nStatus != 1)
        {
            countWeSurvive++;
        }
        if(pstFlayPlane->astUav[i].nWorkStatus == 0
                && pstFlayPlane->astUav[i].nZ >= pstMap->nHLow)
        {
            pstFlayPlane->astUav[i].nGoodsNo = -1;
            pstFlayPlane->astUav[i].nGoodNoTarget = -1;
            vector<int>().swap(pstFlayPlane->astUav[i].nWay);
            pstFlayPlane->astUav[i].nWorkStatus = 4;
        }
        else if(pstFlayPlane->astUav[i].nWorkStatus == 1
                && pstFlayPlane->astUav[i].nStatus != 1)
        {
            int flag = -1;
            for(int j = 0; j < pstMatch->nUavEnemyNum; j++)
            {
                if(pstMatch->astEnemyUav[j].nGoodsNo == pstFlayPlane->astUav[i].nGoodNoTarget
                        && pstMatch->astEnemyUav[j].nGoodsNo != -1
                        && pstMatch->astEnemyUav[j].nStatus != 1)
                {
                    flag = 1;
                    break;
                }
            }
            if(flag == 1
                    && pstFlayPlane->astUav[i].nZ < pstMap->nHLow)
            {
                pstFlayPlane->astUav[i].nGoodsNo = -1;
                pstFlayPlane->astUav[i].nGoodNoTarget = -1;
                vector<int>().swap(pstFlayPlane->astUav[i].nWay);
                pstFlayPlane->astUav[i].nWorkStatus = 0;
            }
            else if(flag == 1
                    && pstFlayPlane->astUav[i].nZ >= pstMap->nHLow)
            {
                pstFlayPlane->astUav[i].nGoodsNo = -1;
                pstFlayPlane->astUav[i].nGoodNoTarget = -1;
                vector<int>().swap(pstFlayPlane->astUav[i].nWay);
                pstFlayPlane->astUav[i].nWorkStatus = 4;
            }
        }
        else if(pstFlayPlane->astUav[i].nWorkStatus == 3
                && pstFlayPlane->astUav[i].nWeUavWhetherChase == 1
                && pstFlayPlane->astUav[i].nStatus != 1)
        {
            int enemyUavNoTemp = pstFlayPlane->astUav[i].nTargetEnemyUav;
            bool whetherServive = false;
            for(int j = 0; j < pstMatch->nUavEnemyNum; j++)
            {
                if(pstMatch->astEnemyUav[j].nNO == enemyUavNoTemp)
                {
                    whetherServive = true;
                }
            }
            if(whetherServive == false)
            {
                vector<int>().swap(pstFlayPlane->astUav[i].nWay);
                pstFlayPlane->astUav[i].nWorkStatus = 0;
                pstFlayPlane->astUav[i].nTargetEnemyUav = -1;
                pstFlayPlane->astUav[i].nWeUavWhetherChase = 0;
            }
        }
        else if(pstFlayPlane->astUav[i].nWorkStatus == 3
                && pstFlayPlane->astUav[i].nWeUavWhetherChase == 1
                && pstFlayPlane->astUav[i].nStatus == 1)
        {
            int enemyUavNoTemp = pstFlayPlane->astUav[i].nTargetEnemyUav;
            pstFlayPlane->astUav[i].nWorkStatus = -1;
            pstFlayPlane->astUav[i].nWeUavWhetherChase = -1;
            enemyBeChased[enemyUavNoTemp] = 0;
        }
        else if(pstFlayPlane->astUav[i].nWorkStatus == 5
                && pstFlayPlane->astUav[i].nX == pstMap->nParkingX
                && pstFlayPlane->astUav[i].nY == pstMap->nParkingY
                && pstFlayPlane->astUav[i].nZ == 0)
        {
            pstFlayPlane->astUav[i].nWorkStatus = 0;
            vector<int>().swap(pstFlayPlane->astUav[i].nWay);
        }
    }
    //标记敌方飞机并排序
    vector<int> nUavEnemySort;
    for(int i = 0; i < pstMatch->nUavEnemyNum; i++)
    {
        nUavEnemySort.push_back(i);
        int EX = pstMatch->astEnemyUav[i].nX;
        int EY = pstMatch->astEnemyUav[i].nY;
        int EZ = pstMatch->astEnemyUav[i].nZ;
        if(pstMatch->astEnemyUav[i].nStatus == 2)
            continue;
        if(pstMatch->astEnemyUav[i].nStatus == 1) ///表示飞机坠毁，更新维0
        {
            space[EZ][EX][EY] = 0;
            continue;
        }
        space[EZ][EX][EY] = 4;// ChangeSpace(space,EX,EY,EZ,pstMap,4);
    }
    int nsize = nUavEnemySort.size(); //待机飞机的数量
    int temp = -1; //临时变量
    if(nsize > 1)
    {
        for(int i = 0; i < nsize - 1; i++)
        {
            for(int j = i + 1; j < nsize; j++)
            {
                if(pstMatch->astEnemyUav[nUavEnemySort[i]].nLoadWeight > pstMatch->astEnemyUav[nUavEnemySort[j]].nLoadWeight)
                {
                    temp = nUavEnemySort[i];
                    nUavEnemySort[i] = nUavEnemySort[j];
                    nUavEnemySort[j] = temp;
                }
            }
        }
    }
    //调试
//    for(int i = 0; i < nUavEnemySort.size(); i++)
//    {
//        cout << "No:" << nUavEnemySort[i].nNO << "   load:" << nUavEnemySort[i].nLoadWeight << "   whether:"
//             << nUavEnemySort[i].nEnemyUavWhetherChase << "   remainElc:" << nUavEnemySort[i].nRemainElectricity
//             << "   weNum:" << pstMatch->nUavWeNum
//             << endl;
//    }
//    cout << "   " << endl;
//    for(int i = 0; i < pstFlayPlane->nUavNum; i++)
//    {
//
//            cout << "No:" << pstFlayPlane->astUav[i].nNO << "   mark:" << enemyBeChased[pstFlayPlane->astUav[i].nTargetEnemyUav]
//                 << "   remainElc:" << pstFlayPlane->astUav[i].nRemainElectricity << "   whether:"
//                 << pstFlayPlane->astUav[i].nWeUavWhetherChase << "   Num:" << pstFlayPlane->nUavNum
//                 << "   workState:" << pstFlayPlane->astUav[i].nWorkStatus
//                 << "   nStatus:" << pstFlayPlane->astUav[i].nStatus << endl;
//
//
//    }
    //以己方飞机去配对敌方飞机
    UAV weUavTem;
    UAV enemyUAVTem;
    int uavNoChase = -1;
    int maxLoad = -1;
    for(int i = 0; i < pstFlayPlane->nUavNum; i++)
    {
        if(pstFlayPlane->astUav[i].nStatus != 1 //活着
                && pstFlayPlane->astUav[i].nLoadWeight <= uavTypeSort[1].nLoadWeight //小飞机追击
                && pstFlayPlane->astUav[i].nWeUavWhetherChase != 1 //己方飞机没有追击
                && (pstFlayPlane->astUav[i].nWorkStatus == 4 || pstFlayPlane->astUav[i].nWorkStatus == 5))
        {
            bool flag = false;
            maxLoad = -1;
            for(int j = 0; j < nUavEnemySort.size(); j++)
            {
                bool flagParking = false;
                if(pstMatch->astEnemyUav[nUavEnemySort[j]].nX == enemyParkingX && pstMatch->astEnemyUav[nUavEnemySort[j]].nY == enemyParkingY && pstMatch->astEnemyUav[nUavEnemySort[j]].nZ == 0)
                {
                    flagParking = true;
                }
                if(flagParking == false && pstMatch->astEnemyUav[nUavEnemySort[j]].nStatus == 0
                        &&  enemyBeChased[pstMatch->astEnemyUav[nUavEnemySort[j]].nNO] == 0) //enemyWe.find(nUavEnemySort[i].nNO) == enemyWe.end() &&
                {
                    if(pstMatch->astEnemyUav[nUavEnemySort[j]].nLoadWeight > maxLoad)
                    {
                        maxLoad = pstMatch->astEnemyUav[nUavEnemySort[j]].nLoadWeight;
                        uavNoChase = pstMatch->astEnemyUav[nUavEnemySort[j]].nNO;
                        flag = true;
                    }
                }
            }
            if(flag == true)
            {
                pstFlayPlane->astUav[i].nTargetEnemyUav = uavNoChase;
                enemyBeChased[uavNoChase] = 1;
                pstFlayPlane->astUav[i].nWorkStatus = 3;
                pstFlayPlane->astUav[i].nWeUavWhetherChase = 1;
            }
        }
    }
    //追击飞机路径更新
    GOODS enemyGoods;
    for(int i = 0; i < pstFlayPlane->nUavNum; i++)
    {
        if(pstFlayPlane->astUav[i].nWorkStatus == 3
                && pstFlayPlane->astUav[i].nWeUavWhetherChase == 1)//寻找目标飞机编号
        {
            int enemyUavNoTemp = pstFlayPlane->astUav[i].nTargetEnemyUav;
            bool whetherServive = false;
            int k = -1;
            for(int j = 0; j < pstMatch->nUavEnemyNum; j++)
            {
                if(pstMatch->astEnemyUav[j].nNO == enemyUavNoTemp)
                {
                    whetherServive = true;
                    k = j;
                }
            }
            if(whetherServive == true)
            {
                if(pstMatch->astEnemyUav[k].nStatus != 1
                        && pstMatch->astEnemyUav[k].nZ < pstMap->nHLow
                        && pstFlayPlane->astUav[i].nZ > pstMatch->astEnemyUav[k].nZ
                        && pstFlayPlane->astUav[i].nX == pstMatch->astEnemyUav[k].nX
                        && pstFlayPlane->astUav[i].nY == pstMatch->astEnemyUav[k].nY
                        && pstMatch->astEnemyUav[k].nGoodsNo != -1)
                {
                    vector<int>().swap(pstFlayPlane->astUav[i].nWay);
                    pstFlayPlane->astUav[i].nEndPoint[0] = pstMatch->astEnemyUav[k].nX;
                    pstFlayPlane->astUav[i].nEndPoint[1] = pstMatch->astEnemyUav[k].nY;
                    pstFlayPlane->astUav[i].nEndPoint[2] = pstMatch->astEnemyUav[k].nZ + 1;
                    Waybfs(&pstFlayPlane->astUav[i], pstMap, pstFlayPlane, pstMatch);
                }
                else if(pstMatch->astEnemyUav[k].nStatus != 1
                        && pstMatch->astEnemyUav[k].nStatus != 2
                        && pstMatch->astEnemyUav[k].nGoodsNo == -1)//目标飞机活着不在雾区无货
                {
                    vector<int>().swap(pstFlayPlane->astUav[i].nWay);
                    pstFlayPlane->astUav[i].nEndPoint[0] = pstMatch->astEnemyUav[k].nX;
                    pstFlayPlane->astUav[i].nEndPoint[1] = pstMatch->astEnemyUav[k].nY;
                    pstFlayPlane->astUav[i].nEndPoint[2] = pstMatch->astEnemyUav[k].nZ;
                    Waybfs(&pstFlayPlane->astUav[i], pstMap, pstFlayPlane, pstMatch);
                }
                else if(pstMatch->astEnemyUav[k].nStatus != 1
                        && pstMatch->astEnemyUav[k].nGoodsNo != -1)//目标飞机或者有货
                {
                    int enemyGoodsNo = pstMatch->astEnemyUav[k].nGoodsNo;
                    bool flag = false;
                    for(int m = 0; m < pstMatch->nGoodsNum; m++)
                    {
                        enemyGoods = pstMatch->astGoods[m];
                        if(enemyGoods.nNO == enemyGoodsNo)
                        {
                            flag = true;
                            break;
                        }
                    }
                    if(flag = true)
                    {
                        vector<int>().swap(pstFlayPlane->astUav[i].nWay);
                        pstFlayPlane->astUav[i].nEndPoint[0] = enemyGoods.nEndX;
                        pstFlayPlane->astUav[i].nEndPoint[1] = enemyGoods.nEndY;
                        pstFlayPlane->astUav[i].nEndPoint[2] = pstMap->nHLow - 1;
                        Waybfs(&pstFlayPlane->astUav[i], pstMap, pstFlayPlane, pstMatch);
                    }
                }
                else if(pstMatch->astEnemyUav[k].nStatus != 1
                        && pstMatch->astEnemyUav[k].nStatus == 2
                        && pstMatch->astEnemyUav[k].nGoodsNo == -1
                        && pstFlayPlane->astUav[i].nEndPoint[0] == pstFlayPlane->astUav[i].nX
                        && pstFlayPlane->astUav[i].nEndPoint[1] == pstFlayPlane->astUav[i].nY
                        && pstFlayPlane->astUav[i].nEndPoint[2] == pstFlayPlane->astUav[i].nZ)
                {
                    vector<int>().swap(pstFlayPlane->astUav[i].nWay);
                    int xFog = 0;
                    int yFog = 0;
                    int zFog = 0;
                    int flagFog = -1;
                    for(int iFog = -2; iFog < 3; iFog++)
                    {
                        for(int jFog = -2; jFog < 3; jFog++)
                        {
                            if(space[pstFlayPlane->astUav[i].nZ][pstFlayPlane->astUav[i].nX + iFog][pstFlayPlane->astUav[i].nY + jFog] == 2)
                            {
                                xFog = pstFlayPlane->astUav[i].nX + iFog;
                                yFog = pstFlayPlane->astUav[i].nY + jFog;
                                zFog = pstFlayPlane->astUav[i].nZ;
                                flagFog = 1;
                                break;
                            }
                        }
                    }
                    if(flagFog == 1)
                    {
                        pstFlayPlane->astUav[i].nEndPoint[0] = xFog;
                        pstFlayPlane->astUav[i].nEndPoint[1] = yFog;
                        pstFlayPlane->astUav[i].nEndPoint[2] = zFog;
                        Waybfs(&pstFlayPlane->astUav[i], pstMap, pstFlayPlane, pstMatch);
                    }
                }
            }
        }
    }
    //待机飞机赋予任务并路径规划
    //小飞机先装载
    vector<int> waitUAV;//待机的飞机
    for(int i = 0; i < pstFlayPlane->nUavNum; i++)
    {
        //如果飞机活着＆＆待机
        if(pstFlayPlane->astUav[i].nStatus != 1
                && pstFlayPlane->astUav[i].nWorkStatus == 4)
        {
            waitUAV.push_back(i);
        }
    }
    int N = waitUAV.size(); //待机飞机的数量
    int t = -1; //临时变量
    for(int i = 0; i < N - 1; i++)
    {
        for(int j = 0; j < N - 1 - i; j++)
        {
            if(pstFlayPlane->astUav[waitUAV[j]].nLoadWeight > pstFlayPlane->astUav[waitUAV[j + 1]].nLoadWeight)
            {
                t = waitUAV[j];
                waitUAV[j] = waitUAV[j + 1];
                waitUAV[j + 1] = t;
            }
        }
    }
    vector <int> waitWay;
    //飞机排序捡货
    for(int i = 0; i < N; i++)
    {
        int whetherSwitchToFive = UavGood(&pstFlayPlane->astUav[waitUAV[i]], pstMatch, pstFlayPlane, pstMap);
        waitWay = pstFlayPlane->astUav[waitUAV[i]].nWay; ///路径坐标
        if(whetherSwitchToFive == -1
                && pstFlayPlane->astUav[i].nStatus != 1
                && pstFlayPlane->astUav[i].nWorkStatus == 4
                && pstFlayPlane->astUav[i].nRemainElectricity < loadWeightCapacity[pstFlayPlane->astUav[i].nLoadWeight])
        {
            pstFlayPlane->astUav[i].nWorkStatus = 5;
        }
    }
    int countOutFly = 0; //统计在外面飞的飞机
    //无任务的飞机路径规划和去充电飞机路径规划
    for(int i = 0; i < pstFlayPlane->nUavNum; i++)
    {
        if(pstFlayPlane->astUav[i].nStatus != 1
                && pstFlayPlane->astUav[i].nZ >= pstMap->nHLow)
        {
            countOutFly++;
        }
        //待机飞机规划路径
        if(pstFlayPlane->astUav[i].nStatus != 1
                && pstFlayPlane->astUav[i].nWorkStatus == 4
                && pstFlayPlane->astUav[i].nWay.size() == 0) //待机飞机去雾区
        {
//            int maxFogNum = fogNum[i % (fogNum.size())];
            //如果到达雾区
            if(pstFlayPlane->astUav[i].nZ == pstMap->nHLow
                    && (pstFlayPlane->astUav[i].nX >= pstMap->astFog[maxFogNum].nX
                        && pstFlayPlane->astUav[i].nX <= (pstMap->astFog[maxFogNum].nX + pstMap->astFog[maxFogNum].nL - 1))
                    && (pstFlayPlane->astUav[i].nY >= pstMap->astFog[maxFogNum] .nY
                        && pstFlayPlane->astUav[i].nY <= (pstMap->astFog[maxFogNum] .nY + pstMap->astFog[maxFogNum].nW - 1)))//绕着雾区边缘走
            {
                if(pstFlayPlane->astUav[i].nX == (pstMap->astFog[maxFogNum].nX + pstMap->astFog[maxFogNum].nL - 1)
                        && pstFlayPlane->astUav[i].nY < (pstMap->astFog[maxFogNum] .nY + pstMap->astFog[maxFogNum].nW - 1))
                {
                    pstFlayPlane->astUav[i].nWay.push_back(pstFlayPlane->astUav[i].nX);
                    pstFlayPlane->astUav[i].nWay.push_back(pstFlayPlane->astUav[i].nY + 1);
                }
                else if(pstFlayPlane->astUav[i].nY == (pstMap->astFog[maxFogNum] .nY + pstMap->astFog[maxFogNum].nW - 1)
                        && pstFlayPlane->astUav[i].nX > (pstMap->astFog[maxFogNum] .nX))
                {
                    pstFlayPlane->astUav[i].nWay.push_back(pstFlayPlane->astUav[i].nX - 1);
                    pstFlayPlane->astUav[i].nWay.push_back(pstFlayPlane->astUav[i].nY);
                }
                else if(pstFlayPlane->astUav[i].nX == (pstMap->astFog[maxFogNum] .nX)
                        && pstFlayPlane->astUav[i].nY > pstMap->astFog[maxFogNum] .nY)
                {
                    pstFlayPlane->astUav[i].nWay.push_back(pstFlayPlane->astUav[i].nX);
                    pstFlayPlane->astUav[i].nWay.push_back(pstFlayPlane->astUav[i].nY - 1);
                }
                else
                {
                    pstFlayPlane->astUav[i].nWay.push_back(pstFlayPlane->astUav[i].nX + 1);
                    pstFlayPlane->astUav[i].nWay.push_back(pstFlayPlane->astUav[i].nY);
                }
                pstFlayPlane->astUav[i].nWay.push_back(pstMap->nHLow);
            }
            else//如果没到雾区
            {
                int fogX = pstMap->astFog[maxFogNum].nX + 1;
                int fogY = pstMap->astFog[maxFogNum].nY + 1;
                int fogL = pstMap->astFog[maxFogNum].nL + fogX - 3;
                int fogW = pstMap->astFog[maxFogNum].nW + fogY - 3;
                pstFlayPlane->astUav[i].nEndPoint[0] = rand() % (fogL - fogX + 1) + fogX; //取得[a,b]的随机整数：rand()%(b-a+1)+a
                pstFlayPlane->astUav[i].nEndPoint[1] = rand() % (fogW - fogY + 1) + fogY;
                pstFlayPlane->astUav[i].nEndPoint[2] = pstMap->nHLow + i % 2;
                Waybfs(&pstFlayPlane->astUav[i], pstMap, pstFlayPlane, pstMatch);
            }
        }
        else if(pstFlayPlane->astUav[i].nStatus != 1
                && pstFlayPlane->astUav[i].nWorkStatus == 5)  //去充电飞机路径规划
        {
            if((pstFlayPlane->astUav[i].nRemainElectricity == loadWeightCapacity[pstFlayPlane->astUav[i].nLoadWeight])
                    || (pstFlayPlane->astUav[i].nX == pstMap->nParkingX && pstFlayPlane->astUav[i].nY == pstMap->nParkingY && pstFlayPlane->astUav[i].nZ == 0))
            {
                if(pstFlayPlane->astUav[i].nZ >= pstMap->nHLow)
                {
                    pstFlayPlane->astUav[i].nWorkStatus = 4;
                }
                else
                {
                    pstFlayPlane->astUav[i].nWorkStatus = 0;
                }
                vector<int>().swap(pstFlayPlane->astUav[i].nWay);
            }
            else
            {
                int flag = 0;
                for(int j = 0; j < pstFlayPlane->nUavNum; j++)
                {
                    if(pstFlayPlane->astUav[j].nX >= pstMap->nParkingX - 4
                            && pstFlayPlane->astUav[j].nY >= pstMap->nParkingY - 4
                            && pstFlayPlane->astUav[j].nX >= pstMap->nParkingX + 4
                            && pstFlayPlane->astUav[j].nY >= pstMap->nParkingY + 4
                            && pstFlayPlane->astUav[j].nZ >= pstMap->nHLow
                            && pstFlayPlane->astUav[j].nWorkStatus == 5)
                    {
                        flag++;
                    }
                }
                if(flag >= 2)
                {
                    pstFlayPlane->astUav[i].nEndPoint[0] = pstMap->nParkingX;
                    pstFlayPlane->astUav[i].nEndPoint[1] = pstMap->nParkingY;
                    pstFlayPlane->astUav[i].nEndPoint[2] = 0;
                    Waybfs(&pstFlayPlane->astUav[i], pstMap, pstFlayPlane, pstMatch);
                }
                else if(flag < 2)
                {
                    pstFlayPlane->astUav[i].nEndPoint[0] = pstMap->nParkingX;
                    pstFlayPlane->astUav[i].nEndPoint[1] = pstMap->nParkingY;
                    pstFlayPlane->astUav[i].nEndPoint[2] = pstMap->nHLow + 2 + i % 2;
                    Waybfs(&pstFlayPlane->astUav[i], pstMap, pstFlayPlane, pstMatch);
                }
            }
        }
    }
//    对所有飞机作排序，小质量飞机先动
//    vector<int> weAllUAVSort;//待机的飞机
//    for(int i = 0; i < pstFlayPlane->nUavNum; i++)
//    {
//        如果飞机活着
//        if(pstFlayPlane->astUav[i].nStatus != 1)
//        {
//            weAllUAVSort.push_back(i);
//        }
//    }
//    int NWeAll = weAllUAVSort.size(); //待机飞机的数量
//    int tempWeAll = -1; //临时变量
//    for(int i = 0; i < NWeAll - 1; i++)
//    {
//        for(int j = 0; j < NWeAll - 1 - i; j++)
//        {
//            if(pstFlayPlane->astUav[weAllUAVSort[j]].nLoadWeight > pstFlayPlane->astUav[weAllUAVSort[j + 1]].nLoadWeight)
//            {
//                tempWeAll = weAllUAVSort[j];
//                weAllUAVSort[j] = weAllUAVSort[j + 1];
//                weAllUAVSort[j + 1] = tempWeAll;
//            }
//        }
//    }
    //飞机步进&&自由飞机路径规划
    for(int i = 0; i < pstFlayPlane->nUavNum; i++)
    {
        space[0][pstMap->nParkingX][pstMap->nParkingY] = 0;
        //如果飞机坠毁
        if(pstFlayPlane->astUav[i].nStatus == 1)
        {
            continue;
        }
        //否则更新地图上的上一刻飞机位置的地图标记
        space[pstFlayPlane->astUav[i].nZ][pstFlayPlane->astUav[i].nX][pstFlayPlane->astUav[i].nY] = pstFlayPlane->astUav[i].nStatus;
        //如果飞机没有到达终点
        if(pstFlayPlane->astUav[i].nWay.size() / 3 != 0)
        {
            //周边异物检测
            //有异物则重新规划路径
            int way = Waybfs(&pstFlayPlane->astUav[i], pstMap, pstFlayPlane, pstMatch);
            //UAV坐标更新
            if(way != 0)//如果有解，否则飞机静止不动
            {
                pstFlayPlane->astUav[i].nX = pstFlayPlane->astUav[i].nWay[0];
                pstFlayPlane->astUav[i].nY = pstFlayPlane->astUav[i].nWay[1];
                pstFlayPlane->astUav[i].nZ = pstFlayPlane->astUav[i].nWay[2];
                for(int j = 0; j < 3; j++)
                    pstFlayPlane->astUav[i].nWay.erase(pstFlayPlane->astUav[i].nWay.begin());
            }
            //如果到达了目标点，更改状态
            if(pstFlayPlane->astUav[i].nZ == pstFlayPlane->astUav[i].nEndPoint[2]
                    && pstFlayPlane->astUav[i].nX == pstFlayPlane->astUav[i].nEndPoint[0]
                    && pstFlayPlane->astUav[i].nY == pstFlayPlane->astUav[i].nEndPoint[1])
            {
                switch(pstFlayPlane->astUav[i].nWorkStatus)
                {
                case 1://如果是去取货，到达后改为送货
                {
                    pstFlayPlane->astUav[i].nGoodsNo = pstFlayPlane->astUav[i].nGoodNoTarget;
                    //pstFlayPlane->astUav[i].nWorkStatus=2;//更改为送货
                    break;
                }
                case 2://如果是去送货，到达后改为自由
                {
                    pstFlayPlane->astUav[i].nWorkStatus = 0;
                    vector<int>().swap(pstFlayPlane->astUav[i].nWay);
                    pstFlayPlane->astUav[i].nGoodNoTarget = -1;
                    pstFlayPlane->astUav[i].nRemainElectricity -= pstFlayPlane->astUav[i].nLoadGoodsWeight;
                    pstFlayPlane->astUav[i].nLoadGoodsWeight = 0;
                    break;
                }
                case 5:
                {
                    pstFlayPlane->astUav[i].nWorkStatus = 0;
                    vector<int>().swap(pstFlayPlane->astUav[i].nWay);
                }
                }
            }
        }
        else//飞机抵达终点（取货＼放货＼待机点）　或　自由　或刚刚将飞机置为待机
        {
            switch(pstFlayPlane->astUav[i].nWorkStatus)
            {
            case 0://如果是自由
            {
                //如果飞机下一步不受阻
                if((space[pstFlayPlane->astUav[i].nZ + 1][pstFlayPlane->astUav[i].nX][pstFlayPlane->astUav[i].nY] == 0
                        || space[pstFlayPlane->astUav[i].nZ + 1][pstFlayPlane->astUav[i].nX][pstFlayPlane->astUav[i].nY] == 2
                        || space[pstFlayPlane->astUav[i].nZ + 1][pstFlayPlane->astUav[i].nX][pstFlayPlane->astUav[i].nY] == 4))
                {
                    if(pstFlayPlane->astUav[i].nZ < pstMap->nHLow
                            && pstFlayPlane->astUav[i].nX == pstMap->nParkingX
                            && pstFlayPlane->astUav[i].nY == pstMap->nParkingY)
                    {
                        int flag = -1;
                        for(int j = 0; j < pstFlayPlane->nUavNum; j++)
                        {
                            if(pstFlayPlane->astUav[j].nStatus != 1
                                    && pstFlayPlane->astUav[j].nWorkStatus == 5
                                    && pstFlayPlane->astUav[j].nX == pstMap->nParkingX
                                    && pstFlayPlane->astUav[j].nY == pstMap->nParkingY
                                    && pstFlayPlane->astUav[j].nZ > 0
                                    && pstFlayPlane->astUav[j].nZ <= pstMap->nHLow)
                            {
                                flag = 1;
                                break;
                            }
                        }
                        if(flag == -1)
                        {
                            if(pstMatch->nTime < 60)
                            {
                                if(pstFlayPlane->astUav[i].nLoadWeight <= uavTypeSort[1].nLoadWeight && pstFlayPlane->astUav[i].nRemainElectricity > 0)
                                {
                                    pstFlayPlane->astUav[i].nGoodsNo = -1;
                                    pstFlayPlane->astUav[i].nGoodNoTarget = -1;
                                    pstFlayPlane->astUav[i].nZ++;
                                }
                                else if(pstFlayPlane->astUav[i].nRemainElectricity == loadWeightCapacity[pstFlayPlane->astUav[i].nLoadWeight])
                                {
                                    pstFlayPlane->astUav[i].nGoodsNo = -1;
                                    pstFlayPlane->astUav[i].nGoodNoTarget = -1;
                                    pstFlayPlane->astUav[i].nZ++;
                                }
                            }
                            else if(pstFlayPlane->astUav[i].nRemainElectricity == loadWeightCapacity[pstFlayPlane->astUav[i].nLoadWeight])
                            {
                                pstFlayPlane->astUav[i].nGoodsNo = -1;
                                pstFlayPlane->astUav[i].nGoodNoTarget = -1;
                                pstFlayPlane->astUav[i].nZ++;
                            }
                        }
                        else if(flag == 1
                                && pstFlayPlane->astUav[i].nZ > 0
                                && space[pstFlayPlane->astUav[i].nZ - 1][pstFlayPlane->astUav[i].nX][pstFlayPlane->astUav[i].nY] != 3)
                        {
                            pstFlayPlane->astUav[i].nGoodsNo = -1;
                            pstFlayPlane->astUav[i].nGoodNoTarget = -1;
                            pstFlayPlane->astUav[i].nZ--;
                        }
                    }
                    else
                    {
                        pstFlayPlane->astUav[i].nGoodsNo = -1;
                        pstFlayPlane->astUav[i].nGoodNoTarget = -1;
                        pstFlayPlane->astUav[i].nZ++;
                    }
                    break;
                }
            }
            case 1://如果是去取货，到达后改为送货
            {
                if(pstFlayPlane->astUav[i].nGoodsNo != -1) //如果被自己捡起
                {
                    pstFlayPlane->astUav[i].nWorkStatus = 2;
                    //目的地更改
                    pstFlayPlane->astUav[i].nEndPoint[0] = pstFlayPlane->astUav[i].nEndPoint[3];
                    pstFlayPlane->astUav[i].nEndPoint[1] = pstFlayPlane->astUav[i].nEndPoint[4];
                    pstFlayPlane->astUav[i].nEndPoint[2] = pstFlayPlane->astUav[i].nEndPoint[5];
                    int aaa = pstFlayPlane->astUav[i].nEndPoint[0];
                    aaa = pstFlayPlane->astUav[i].nEndPoint[1];
                    int x = pstFlayPlane->astUav[i].nX;
                    int y = pstFlayPlane->astUav[i].nY;
                    int z = pstFlayPlane->astUav[i].nZ;
                    //更新规划路径
                    //WayPlan(&pstFlayPlane->astUav[i],pstMap);
                    int kk = Waybfs(&pstFlayPlane->astUav[i], pstMap, pstFlayPlane, pstMatch);
                    //UAV坐标更新
                    pstFlayPlane->astUav[i].nX = pstFlayPlane->astUav[i].nWay[0];
                    pstFlayPlane->astUav[i].nY = pstFlayPlane->astUav[i].nWay[1];
                    pstFlayPlane->astUav[i].nZ = pstFlayPlane->astUav[i].nWay[2];
                    int sizee = pstFlayPlane->astUav[i].nWay.size();
                    for(int k = 0; k < 3; k++)
                        pstFlayPlane->astUav[i].nWay.erase(pstFlayPlane->astUav[i].nWay.begin());
                }
                else //如果被敌方捡起
                {
                    int r = pstFlayPlane->astUav[i].nGoodsNo;
                    pstFlayPlane->astUav[i].nWorkStatus = 0;
                    pstFlayPlane->astUav[i].nGoodNoTarget = -1;
                    vector<int>().swap(pstFlayPlane->astUav[i].nWay);
                }
                break;
            }
            case 2://如果是去送货，到达后改为自由
            {
                //pstFlayPlane->astUav[i].nGoodNoTarget = -1;
                break;
            }
            case 3://如果是攻击，到达后再说？？？？
            {
                break;
            }
            case 4://如果是待机，到达后不操作
            {
                break;
            }
            }
        }
        //更新地图上的此刻飞机位置的地图标记３
        space[pstFlayPlane->astUav[i].nZ][pstFlayPlane->astUav[i].nX][pstFlayPlane->astUav[i].nY] = 3;
        space[0][pstMap->nParkingX][pstMap->nParkingY] = 0;
    }
//释放飞机坐标
//标记己方飞机
    for(int i = 0; i < pstMatch->nUavWeNum; i++)
    {
        int PX = pstMatch->astWeUav[i].nX;
        int PY = pstMatch->astWeUav[i].nY;
        int PZ = pstMatch->astWeUav[i].nZ;
        if(pstMatch->astWeUav[i].nStatus != 1) ///表示飞机坠毁，更新维0
        {
            space[PZ][PX][PY] = pstMatch->astWeUav[i].nStatus;
        }
    }
//标记敌方飞机
    for(int i = 0; i < pstMatch->nUavEnemyNum; i++)
    {
        int EX = pstMatch->astEnemyUav[i].nX;
        int EY = pstMatch->astEnemyUav[i].nY;
        int EZ = pstMatch->astEnemyUav[i].nZ;
        if(pstMatch->astEnemyUav[i].nStatus == 2)
            continue;
        if(pstMatch->astEnemyUav[i].nStatus != 1) ///表示飞机未坠毁，更新维0
        {
            space[EZ][EX][EY] = pstMatch->astEnemyUav[i].nStatus;// ChangeSpace(space,EX,EY,EZ,pstMap,4);
        }
    }
//更新飞机电量
//获取己方小载重飞机数量
    int countWeSmallUav = 0;
    countWeSurvive = 0;
    int onlySurvive = -1;
    for(int i = 0; i < pstFlayPlane->nUavNum; i++)
    {
        if(pstFlayPlane->astUav[i].nLoadWeight == 20 && pstFlayPlane->astUav[i].nStatus != 1)
        {
            countWeSmallUav++;
        }
        if(pstFlayPlane->astUav[i].nStatus != 1)
        {
            countWeSurvive++;
            onlySurvive = i;
        }
        if(pstFlayPlane->astUav[i].nX == pstMap->nParkingX
                && pstFlayPlane->astUav[i].nY == pstMap->nParkingY
                && pstFlayPlane->astUav[i].nZ == 0)
        {
            pstFlayPlane->astUav[i].nRemainElectricity += loadWeightCharge[pstFlayPlane->astUav[i].nLoadWeight];
            if(pstFlayPlane->astUav[i].nRemainElectricity >= loadWeightCapacity[pstFlayPlane->astUav[i].nLoadWeight])
            {
                pstFlayPlane->astUav[i].nRemainElectricity = loadWeightCapacity[pstFlayPlane->astUav[i].nLoadWeight];
            }
        }
        else if(pstFlayPlane->astUav[i].nGoodsNo != -1)
        {
            pstFlayPlane->astUav[i].nRemainElectricity -= pstFlayPlane->astUav[i].nLoadGoodsWeight;
        }
    }
//购买飞机
    if(pstMatch->nWeValue >= uavTypeSort[0].nValue && pstFlayPlane->nUavNum < 50)
    {
        if(countWeSmallUav >= (nUavEnemySort.size() + 5) && pstMatch->nWeValue >= uavTypeSort[1].nValue)
        {
            if(pstMatch->nWeValue >= uavTypeSort[1].nValue * 3)
            {
                pstFlayPlane->nPurchaseNum = (pstMatch->nWeValue - 3 * uavTypeSort[1].nValue) / uavTypeSort[0].nValue + 3;
                for(int i = 0; i < 3; i++)
                {
                    for(int j = 0; j < 8; j++)
                    {
                        pstFlayPlane->szPurchaseType[i][j] = uavTypeSort[1].szType[j];
                    }
                }
                for(int i = 3; i < pstFlayPlane->nPurchaseNum; i++)
                {
                    for(int j = 0; j < 8; j++)
                    {
                        pstFlayPlane->szPurchaseType[i][j] = uavTypeSort[0].szType[j];
                    }
                }
            }
            else if(pstMatch->nWeValue >= uavTypeSort[1].nValue * 2)
            {
                pstFlayPlane->nPurchaseNum = (pstMatch->nWeValue - 2 * uavTypeSort[1].nValue) / uavTypeSort[0].nValue + 2;
                for(int i = 0; i < 2; i++)
                {
                    for(int j = 0; j < 8; j++)
                    {
                        pstFlayPlane->szPurchaseType[i][j] = uavTypeSort[1].szType[j];
                    }
                }
                for(int i = 2; i < pstFlayPlane->nPurchaseNum; i++)
                {
                    for(int j = 0; j < 8; j++)
                    {
                        pstFlayPlane->szPurchaseType[i][j] = uavTypeSort[0].szType[j];
                    }
                }
            }
            else if(pstMatch->nWeValue >= uavTypeSort[1].nValue)
            {
                pstFlayPlane->nPurchaseNum = (pstMatch->nWeValue - 1 * uavTypeSort[1].nValue) / uavTypeSort[0].nValue + 1;
                for(int i = 0; i < 1; i++)
                {
                    for(int j = 0; j < 8; j++)
                    {
                        pstFlayPlane->szPurchaseType[i][j] = uavTypeSort[1].szType[j];
                    }
                }
                for(int i = 1; i < pstFlayPlane->nPurchaseNum; i++)
                {
                    for(int j = 0; j < 8; j++)
                    {
                        pstFlayPlane->szPurchaseType[i][j] = uavTypeSort[0].szType[j];
                    }
                }
            }
        }
        else
        {
            pstFlayPlane->nPurchaseNum = (pstMatch->nWeValue / uavTypeSort[0].nValue);
            for(int i = 0; i < pstFlayPlane->nPurchaseNum; i++)
            {
                for(int j = 0; j < 8; j++)
                {
                    pstFlayPlane->szPurchaseType[i][j] = uavTypeSort[0].szType[j];
                }
            }
        }
    }
//只剩一架飞机还在追击，则更改状态
    if(countWeSurvive == 1 && onlySurvive != -1)
    {
        if(pstFlayPlane->astUav[onlySurvive].nWorkStatus == 3 && pstFlayPlane->astUav[onlySurvive].nZ >= pstMap->nHLow)
        {
            if(pstFlayPlane->astUav[onlySurvive].nEndPoint[2] >= pstMap->nHLow)
            {
                vector<int>().swap(pstFlayPlane->astUav[onlySurvive].nWay);
                pstFlayPlane->astUav[onlySurvive].nWorkStatus = 4;
                pstFlayPlane->astUav[onlySurvive].nTargetEnemyUav = -1;
                pstFlayPlane->astUav[onlySurvive].nWeUavWhetherChase = 0;
                enemyBeChased[pstFlayPlane->astUav[onlySurvive].nTargetEnemyUav] = 0;
            }
        }
    }
    vector<int>().swap(nUavEnemySort);
    vector<int>().swap(waitUAV);//待机的飞机
}
int main(int argc, char *argv[])
{
#ifdef OS_WINDOWS
    // windows下，需要进行初始化操作
    WSADATA wsa;
    if(WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
    {
        printf("WSAStartup failed\n");
        return false;
    }
#endif
    char        szIp[64] = { 0 };
    int         nPort = 0;
    char        szToken[128] = { 0 };
    int         nRet = 0;
    OS_SOCKET   hSocket;
    char        *pRecvBuffer = NULL;
    char        *pSendBuffer = NULL;
    int         nLen = 0;
    //本地调试去掉这个
    if(argc != 4)
    {
        printf("error arg num\n");
        return -1;
    }
//    解析参数
    strcpy(szIp, argv[1]);
    nPort = atoi(argv[2]);
    strcpy(szToken, argv[3]);
//    strcpy(szIp, "123.56.15.18");
//    nPort = 31186;
//    strcpy(szToken, "82418eac-33fd-4a7a-a0c2-11e8a9cc298b");
    srand((int)time(0));
    printf("server ip %s, prot %d, token %s\n", szIp, nPort, szToken);
    // 开始连接服务器
    nRet = OSCreateSocket(szIp, (unsigned int)nPort, &hSocket);
    if(nRet != 0)
    {
        printf("connect server error\n");
        return nRet;
    }
    // 分配接受发送内存
    pRecvBuffer = (char*)malloc(MAX_SOCKET_BUFFER);
    if(pRecvBuffer == NULL)
    {
        return -1;
    }
    pSendBuffer = (char*)malloc(MAX_SOCKET_BUFFER);
    if(pSendBuffer == NULL)
    {
        free(pRecvBuffer);
        return -1;
    }
    memset(pRecvBuffer, 0, MAX_SOCKET_BUFFER);
    // 接受数据  连接成功后，Judger会返回一条消息：
    nRet = RecvJuderData(hSocket, pRecvBuffer);
    if(nRet != 0)
    {
        return nRet;
    }
    // json 解析
    // 获取头部
    CONNECT_NOTICE  stNotice;
    nRet = ParserConnect(pRecvBuffer + SOCKET_HEAD_LEN, &stNotice);
    if(nRet != 0)
    {
        return nRet;
    }
    // 生成表明身份的json
    TOKEN_INFO  stToken;
    strcpy(stToken.szToken, szToken);  // 如果是调试阶段，请输入你调试的token，在我的对战中获取，
    // 实际比赛，不要传入调试的，按照demo写的，有服务器调用传入。
    strcpy(stToken.szAction, "sendtoken");
    memset(pSendBuffer, 0, MAX_SOCKET_BUFFER);
    nRet = CreateTokenInfo(&stToken, pSendBuffer, &nLen);
    if(nRet != 0)
    {
        return nRet;
    }
    // 选手向裁判服务器表明身份(Player -> Judger)
    nRet = SendJuderData(hSocket, pSendBuffer, nLen);
    if(nRet != 0)
    {
        return nRet;
    }
    //身份验证结果(Judger -> Player)　
    memset(pRecvBuffer, 0, MAX_SOCKET_BUFFER);
    nRet = RecvJuderData(hSocket, pRecvBuffer);
    if(nRet != 0)
    {
        return nRet;
    }
    // 解析验证结果的json
    TOKEN_RESULT      stResult;
    nRet = ParserTokenResult(pRecvBuffer + SOCKET_HEAD_LEN, &stResult);
    if(nRet != 0)
    {
        return 0;
    }
    // 是否验证成功
    if(stResult.nResult != 0)
    {
        printf("token check error\n");
        return -1;
    }
    // 选手向裁判服务器表明自己已准备就绪(Player -> Judger)
    READY_PARAM     stReady;
    strcpy(stReady.szToken, szToken);
    strcpy(stReady.szAction, "ready");
    memset(pSendBuffer, 0, MAX_SOCKET_BUFFER);
    nRet = CreateReadyParam(&stReady, pSendBuffer, &nLen);
    if(nRet != 0)
    {
        return nRet;
    }
    nRet = SendJuderData(hSocket, pSendBuffer, nLen);
    if(nRet != 0)
    {
        return nRet;
    }
    //对战开始通知(Judger -> Player)　
    memset(pRecvBuffer, 0, MAX_SOCKET_BUFFER);
    nRet = RecvJuderData(hSocket, pRecvBuffer);
    if(nRet != 0)
    {
        return nRet;
    }
    // 解析数据
    //Mapinfo 结构体可能很大，不太适合放在栈中，可以定义为全局或者内存分配
    MAP_INFO            *pstMapInfo;
    MATCH_STATUS        *pstMatchStatus;
    FLAY_PLANE          *pstFlayPlane;
    pstMapInfo = (MAP_INFO *)malloc(sizeof(MAP_INFO));
    if(pstMapInfo == NULL)
    {
        return -1;
    }
    pstMatchStatus = (MATCH_STATUS *)malloc(sizeof(MATCH_STATUS));
    if(pstMapInfo == NULL)
    {
        return -1;
    }
    pstFlayPlane = (FLAY_PLANE *)malloc(sizeof(FLAY_PLANE));
    if(pstFlayPlane == NULL)
    {
        return -1;
    }
    memset(pstMapInfo, 0, sizeof(MAP_INFO));
    memset(pstMatchStatus, 0, sizeof(MATCH_STATUS));
    memset(pstFlayPlane, 0, sizeof(FLAY_PLANE));
    nRet = ParserMapInfo(pRecvBuffer + SOCKET_HEAD_LEN, pstMapInfo);
    if(nRet != 0)
    {
        return nRet;
    }
    // 第一次把无人机的初始赋值给flayplane
    pstFlayPlane->nPurchaseNum = 0;
    pstFlayPlane->nUavNum = pstMapInfo->nUavNum;
    for(int i = 0; i < pstMapInfo->nUavNum; i++)
    {
        pstFlayPlane->astUav[i] = pstMapInfo->astUav[i];
    }
    space = (int***)new int**[pstMapInfo->nMapZ];//指向200个二级指针
    for(int i = 0; i < pstMapInfo->nMapZ; i++)
    {
        space[i] = (int**)new int *[pstMapInfo->nMapX];
        for(int j = 0; j < pstMapInfo->nMapX; j++)
            space[i][j] = (int*)new int [pstMapInfo->nMapY];
    }
    for(int z = 0; z < pstMapInfo->nMapZ; z++) ///初始化
        for(int x = 0; x < pstMapInfo->nMapX; x++)
            for(int y = 0; y < pstMapInfo->nMapY; y++)
                space[z][x][y] = 0;
    dis = (int**)new int*[pstMapInfo->nMapX];
    LastDis = (int**)new int*[pstMapInfo->nMapX];
    for(int j = 0; j < pstMapInfo->nMapY; j++)
    {
        dis[j] = (int*)new int [pstMapInfo->nMapY];
        LastDis[j] = (int*)new int [pstMapInfo->nMapY];
    }
    for(int x = 0; x < pstMapInfo->nMapX; x++)
        for(int y = 0; y < pstMapInfo->nMapY; y++)
        {
            LastDis[x][y] = INF;
            dis[x][y] = INF;
        }
    for(int i = 0; i < pstMapInfo->nUavPriceNum; i++)
    {
        loadWeightCapacity[pstMapInfo->astUavPrice[i].nLoadWeight] = pstMapInfo->astUavPrice[i].nCapacity;
        loadWeightCharge[pstMapInfo->astUavPrice[i].nLoadWeight] = pstMapInfo->astUavPrice[i].nCharge;
        cout << "type:" << pstMapInfo->astUavPrice[i].szType << "  value:" << pstMapInfo->astUavPrice[i].nValue
             << "   load:" << pstMapInfo->astUavPrice[i].nLoadWeight << "   capa:" << pstMapInfo->astUavPrice[i].nCapacity
             << "   charge:" << pstMapInfo->astUavPrice[i].nCharge << endl;
    }
    Dg = (int***)new int**[pstMapInfo->nMapZ];//指向200个二级指针
    for(int i = 0; i < pstMapInfo->nMapZ; i++)
    {
        Dg[i] = (int**)new int *[pstMapInfo->nMapX];
        for(int j = 0; j < pstMapInfo->nMapX; j++)
            Dg[i][j] = (int*)new int [pstMapInfo->nMapY];
    }
    //memset(space,0,sizeof(space));///出错
    // 根据服务器指令，不停的接受发送数据
    static int num = 0;
    while(1)
    {
        // 进行当前时刻的数据计算, 填充飞行计划结构体，注意：0时刻不能进行移动，即第一次进入该循环时
        if(pstMatchStatus->nTime != 0)
        {
            AlgorithmCalculationFun(pstMapInfo, pstMatchStatus, pstFlayPlane);
            num++;
        }
        //0时刻进行电量更新
//        for(int i = 0; i < pstFlayPlane->nUavNum; i++)
//        {
//            pstFlayPlane->astUav[i].nRemainElectricity += loadWeightCharge[pstFlayPlane->astUav[i].nLoadWeight];
//        }
//        cout<<pstFlayPlane->astUav[1].nRemainElectricity<<endl;
        //发送飞行计划结构体
        memset(pSendBuffer, 0, MAX_SOCKET_BUFFER);
        nRet = CreateFlayPlane(pstFlayPlane, szToken, pSendBuffer, &nLen);
        if(nRet != 0)
        {
            return nRet;
        }
        nRet = SendJuderData(hSocket, pSendBuffer, nLen);
        if(nRet != 0)
        {
            return nRet;
        }
        printf("%s\n", pSendBuffer);
        // 接受当前比赛状态
        memset(pRecvBuffer, 0, MAX_SOCKET_BUFFER);
        nRet = RecvJuderData(hSocket, pRecvBuffer);
        if(nRet != 0)
        {
            return nRet;
        }
        // 解析
        nRet = ParserMatchStatus(pRecvBuffer + SOCKET_HEAD_LEN, pstMatchStatus);
        if(nRet != 0)
        {
            return nRet;
        }
        if(pstMatchStatus->nMacthStatus == 1)
        {
            // 比赛结束
            printf("game over, we value %d, enemy value %d\n", pstMatchStatus->nWeValue, pstMatchStatus->nEnemyValue);
            return 0;
        }
    }
    // 关闭socket
    OSCloseSocket(hSocket);
    // 资源回收
    free(pRecvBuffer);
    free(pSendBuffer);
    free(pstMapInfo);
    free(pstMatchStatus);
    free(pstFlayPlane);
    for(int j = 0; j < pstMapInfo->nMapZ; j++)
    {
        for(int k = 0; k < pstMapInfo->nMapX; ++k)
        {
            for(int i = 0; i < pstMapInfo->nMapY; ++i)
            {
                delete []space[k][i];
            }
            delete [] space[k];
        }
        delete[]space;
    }
    return 0;
}
