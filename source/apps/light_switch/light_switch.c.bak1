/***********************************************************************************
  Filename: demo_code.c

  Description:  底层部分不会写，先把路由部分的逻辑框架写了好了

***********************************************************************************/

/***********************************************************************************
* INCLUDES
*/
#include <hal_lcd.h>
#include <hal_led.h>
#include <hal_joystick.h>
#include <hal_assert.h>
#include <hal_board.h>
#include <hal_int.h>
#include "hal_mcu.h"
#include "hal_button.h"
#include "hal_rf.h"
#include "util_lcd.h"
#include "basic_rf.h"

/***********************************************************************************
* CONSTANTS
*/
// Application parameters
#define RF_CHANNEL                25      // 2.4 GHz RF channel

// BasicRF address definitions
#define DEFAULT_PAN_ID                0x0000
//地址如何定义的？？？
#define CENTER_ADDR           0x2520
#define MEMBER_ADDR           0xBEEF
#define SERVER_ADDR           0x0000 //服务器的地址为0，即id为0。感觉不用定义，就是服务器

#define MAC_ADDR              0x0000 //MAC地址，等同于basicRfCfg_t中的myAddr，需要人工设置1，2，3，4，。。。
#define APP_PAYLOAD_LENGTH        1
#define LIGHT_TOGGLE_CMD          0

// Application states
#define IDLE                      0
#define SEND_CMD                  1

//设备角色
#define NONE    0
#define ROUTE   1

// MEMBER和CENTER都属于DEVICE
#define MEMBER  1
#define CENTER  2

#define DEVICE  0
#define SERVER  1

//帧类型
#define  BROADCAST 1
#define  ROUTEACK 2
#define  REQUEST 3
#define  REPLY 4
#define  BEACON 5
#define  COMMUNICATEACK 6
#define  DATAPACK 7

//参数
#define MAX_CLUSTER 3   //簇容量
#define TOTAL_NODES 20 //全网节点最大数量


//定时周期
#define CREATE_CLUSTER_LISTEN_TIME  500     //建簇监听时限
#define CREATE_CLUSTER_BEACON 100  //建簇期间beacon发送周期
#define WAIT_ACK_TIME   500
#define TBEACON 10000   //节点通讯beacon周期
#define START_ROUTE_TIME 60000  //服务器发送路由广播时间
#define SLOT 5 //每个时隙 5 ms

/***********************************************************************************
* LOCAL VARIABLES
*/
static uint8 pTxData[APP_PAYLOAD_LENGTH];
static uint8 pRxData[APP_PAYLOAD_LENGTH];
static basicRfCfg_t basicRfConfig;

// Mode menu
static menuItem_t pMenuItems[] =
{
#ifdef ASSY_EXP4618_CC2420//???
  // Using Softbaugh 7-seg display
  " L S    ", SEVICE,
  " LIGHT  ", SERVER
#else
  // SRF04EB and SRF05EB
  "DEVICE",    DEVICE,
  "SERVER",     SERVER
#endif
};

static menu_t pMenu =
{
  pMenuItems,
  N_ITEMS(pMenuItems)
};


#ifdef SECURITY_CCM
// Security key
static uint8 key[]= {
    0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7,
    0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf,
};
#endif

/*
定义帧格式，参考协议设计文档中帧格式,这部分应该放到basic_rf.c中
*/

//路由广播
typedef struct {
    uint8 TYPE;
    uint32 SA;
    uint32 DA;
    uint32 SERVER_ADDR;
} Route_Broadcast;
//路由回复
typedef struct {
    uint8 TYPE;
    uint8 HOP;
    uint32 SA;
    uint32 Destination;
    uint32 DA;
    uint32 LIFETIME;
}Route_ACK;
//入簇请求
typedef struct {
    uint8 TYPE;
    uint16 LIFETIME;
    uint32 RA;
    uint32 SA;
    uint32 FCS;
}Cluster_Request;
//入簇回复
typedef struct {
    uint8 TYPE;
    uint16 LIFETIME;
    uint32 RA;
    uint32 SA;
    uint8 CLUSTER_ID;
    uint32 FCS;
}Cluster_Reply;
//beacon帧
typedef struct {
    uint8 TYPE;
    uint16 LIFETIME;
    uint32 SA;
    uint16 INTERVAL;
    //帧体
    uint16 B_CFP;
    uint16 T_SLOT;
    uint16 B_CAP;
    uint16 B_SLEEP;
    uint8 CENTER_CHANGE_FIELD;
    uint8 RECEIVE_ACK[MAX_CLUSTER];
    uint32 FCS;
}Beacon;
//应答帧
typedef struct {
    uint8 TYPE;
    uint16 LIFETIME;
    uint32 RA;
    uint32 FCS;
}Communication_ACK;
//数据帧
typedef struct {
    uint8 TYPE;
    uint8 IF_C2C;//控制位，判断数据是簇间通信还是簇内通信
    uint16 LIFETIME;
    uint32 RA;
    uint32 SA;
    uint32 Data_Body[MAX_CLUSTER];
    uint32 FCS;
}Data_Pack;

//用来统一各种类型的帧，以方便转化为比特位发送
typedef struct {
    uint8 TYPE;//指示帧类型
    Route_Broadcast     route_broadcast;
    Route_ACK   route_ack; 
    Cluster_Request     cluster_req;
    Cluster_Reply   cluster_rep;
    Beacon  beacon;
    Communication_ACK   c_ack;
    Data_Pack   data_pack;
}Packet;

typedef struct {
    uint32 MAC_ID;//本设备的MAC地址
    uint8 IF_ROUTE;
    uint8 IF_SERVER;
    uint8 IF_CENTER;
    uint8 IF_CLUSTER_FULL;
    uint8 IF_HAVEROUTE;
    uint8 IF_FRIST_ROUTE;
    uint8 IF_ADD_CLUSTER;
    uint32 Cluster_Member[MAX_CLUSTER];//簇内节点地址，适用于本设备是簇头的情况
    uint8 Cluster_Member_Num;//簇内节点数，适用于本设备是簇头的情况
    uint8 Cluster_ID;//簇内ID，适用于本设备是簇成员的情况
    uint32 Cluster_Center;//簇首ID，适用于本设备是簇成员的情况
    uint32 cluster_membet_access_fail[MAX_CLUSTER];//簇内成员接收确认，适用于本设备为簇头,判断每个簇成员的数据是否接收到
    uint32 SERVER_ID;
    uint32 Next_Route;
    uint32 ACC_Node_ID[TOTAL_NODES];//接入网络的节点总地址，适用于服务器
    uint8 ACC_Number;//接入网络的节点数，适用于服务器
    uint8 IF_GET_ACK;//是否接收到ACK；
}Node_Attribute;//节点属性

/***********************************************************************************
* LOCAL FUNCTIONS
*/
static void appMemmber();
static void appCenter();
static void Set_Up_Cluster();//建簇过程
static void appServer();
static uint8 appSelectRole(void);
static Packet Rx2Pack(RxData *);//函数需要补充修改定义，传入参数为缓存中接收到的消息序列，将缓存中消息序列转化为帧,下面有设计思路
static void Send_Pack(Packet *);//函数需要补充定义，传入参数为帧，将需要发送的帧发送出去
static void ReceiveHandle(Packet * ,Node_Attribute * );//函数需要补充定义，用来针对接收到不同的帧信心不同处理（如发送，休眠），下面有设计思路
static uint8 Set_Timer(uint 8);//定时器，当定时器时间到达时返回值为0，未定义，需要补充或者修改
//**********************************************************************************************************
//Rx2Pack ()设计思路：
static Packet Rx2Pack(RxData *rxdata)//RxDate为收到的二进制消息序列
{
    Packet send_pack;
    //读取rx中第一个字节，获得帧的类型
   send_pack.TYPE  = rxdata[0:8];//此处为python写法，要修改。。。
    switch(Packet.TYPE)
    {
        case BROADCAST:
        send_pack.Broadcast.TYPE=rxdata[0:8];
        send_pack.Broadcast.DA = rxdata[8:40];
        send_pack.Broadcast.SERVER_ADDR = rxdata[40:72];
        break;

        case ROUTEACK:
        send_pack.route_ack.TYPE = rxdata[0:8];
        send_pack.route_ack.HOP = rxdata[0:8];//后面直接复制了，下标没改，总之是读取接收缓存中对应的部分
        send_pack.route_ack.SA = rxdata[0:8];
        send_pack.route_ack.Destination = rxdata[0:8];
        send_pack.route_ack.DA = rxdata[0:8];
        send_pack.route_ack.LIFETIME = rxdata[0:8];
        break;

        case REQUEST:
        send_pack.cluster_req.TYPE = rxdata[0:8];
        send_pack.cluster_req.LIFETIME = rxdata[0:8];
        send_pack.cluster_req.RA = rxdata[0:8];
        send_pack.cluster_req.SA = rxdata[0:8];
        send_pack.cluster_req.FCS = rxdata[0:8];
        break;

        case REPLY:
        send_pack.cluster_rep.TYPE = rxdata[0:8];
        send_pack.cluster_rep.LIFETIME = rxdata[0:8];
        send_pack.cluster_rep.SA = rxdata[0:8];
        send_pack.cluster_rep.RA = rxdata[0:8];
        send_pack.cluster_rep.CLUSTER_ID = rxdata[0:8];
        send_pack.cluster_rep.FCS = rxdata[0:8];
        break;

        case BEACON:
        send_pack.beacon.TYPE = rxdata[0:8];
        send_pack.beacon.LIFETIME = rxdata[0:8];
        send_pack.beacon.SA = rxdata[0:8];
        send_pack.beacon.INTERVAL = rxdata[0:8];
        send_pack.beacon.B_CFP = rxdata[0:8];
        send_pack.beacon.T_SLOT = rxdata[0:8];
        send_pack.beacon.B_CAP = rxdata[0:8];
        send_pack.beacon.B_SLEEP = rxdata[0:8];
        send_pack.beacon.CENTER_CHANGE_FIELD = rxdata[0:8];
        uint8 i = 0;
        while(i<MAX_CLUSTER)
        {
            send_pack.beacon.RECEIVE_ACK[i] = rxdata[0 + 8*i:8*(i+1)];
            i++;
        }
        send_pack.beacon.FCS = rxdata[0:8];
        break;

        case COMMUNICATEACK:
        send_pack.c_ack.TYPE = rxdata[0:8];
        send_pack.c_ack.LIFETIME = rxdata[0:8];
        send_pack.c_ack.RA = rxdata[0:8];
        send_pack.c_ack.FCS = rxdata[0:8];
        break;

        case DATEPACK:
        send_pack.data_pack.TYPE = rxdata[0:8];
        send_pack.data_pack.LIFETIME = rxdata[0:8];
        send_pack.IF_C2C = rxdata[0:8];
        send_pack.data_pack.RA = rxdata[0:8];
        send_pack.data_pack.SA = rxdata[0:8];
        uint8 i = 0;
        while(i<MAX_CLUSTER)
        {
            send_pack.data_pack.Data_Body[i] = rxdata[0 + 8*i:8*(i+1)];
            i++;
        }
        send_pack.data_pack.FCS = rxdata[0:8];
        break;

        default:
        break;
    }

    return send_pack;
}

//ReceiveHandle() 设计思路：
static void Route_Broadcast_Handle(Packet *packet,Node_Attribute *attribute) //当接收路由广播帧时采取得出操作
{
    if(attribute.Next_Route == 0 && attribute.IF_ROUTE == ROUTE)
    {
        attribute.IF_HAVEROUTE = 1;
        attribute.Next_Route = packet.route_broadcast.SA;
        attribute.SERVER_ID = packet.route_broadcast.SERVER_ADDR;
        Send_Pack send_packet;
        if(attribute.IF_FRIST_ROUTE == 0){//如果本设备是第一次接收到路由广播，则向上传输一个路由回复
            Route_ACK route_ack;
            Send_Pack send_packet;
            route_ack.TYPE = ROUTEACK;
            route_acK.SA = attribute.MAC_ID;
            route_ack.Destination = attribute.Next_Route;
            route_ack.DA = packet.route_broadcast.SERVER_ADDR;

            send_packet.route_ack = route_ack;
            send_packet.TYPE = ROUTEACK;

            Send_Pack(send_packet);//发送路由回复
        }
    }

    //！！！这里需要加一个函数防止短时间内重复接收到路由广播然后反复发送
    //发送路由广播
    Route_Broadcast broadcast;
    broadcast.SA = attribute.MAC_ID;
    broadcast.SERVER_ADDR = packet.SERVER_ADDR;
    send_packet.route_broadcast = broadcast;
    send_packet.TYPE = BROADCAST;

    Send_Pack(send_packet);//广播路由广播,Broadcast_Pack函数需要定义
}

static void Route_Ack_Handle(Packet *packet,Node_Attribute *attribute)//如果接收到路由回复
{
    if(attribute.MAC_ID == packet.route_ack.DA)//如果当前节点是目的节点(当前节点为服务器)
    {   uint8 IF_IN_SERVER = 0;//判断该节点是否已在服务器中，若不在，则添进服务器属性中
        for(uint16 i = 0; i < attribute.ACC_Number; i++)
        {
           if(attribute.ACC_Node_ID[i] == packet.route_ack.SA) IF_IN_SERVER = 1;
        }

        if(IF_IN_SERVER == 0)//如果服务器中不含该路由点，将其添加到服务器中
        {
            attribute.ACC_Node_ID[attribute.ACC_Number] = packet.route_ack.SA;
            attribute.ACC_Number++;
        }
    }

    else if(attribute.MAC_ID == packet.route_ack.Destination) //如果当前设备只是中介节点
    {
            Route_ACK route_ack;
            Send_Pack send_packet;
            route_ack.TYPE = ROUTEACK;
            route_acK.SA = packet.route_ack.SA;
            route_ack.Destination = attribute.Next_Route;
            route_ack.DA = packet.route_broadcast.SERVER_ADDR;

            send_packet.route_ack = route_ack;
            send_packet.TYPE = ROUTEACK;

            Send_Pack(send_packet);//发送路由回复
    }
}

static void Route_Request_Handle(Packet *packet,Node_Attribute *attribute)//如果接收到的是入簇请求帧
{
    if(attribute.IF_CENTER == CENTER && attribute.MAC_ID == packet.route_request.RA)
    {
        if(attribute.Cluster_Member_Num < MAX_CLUSTER)
        {
            Send_Pack send_packet;
            attribute.Cluster_Member[Cluster_Member_Num] = packet.cluster_reply.SA;
            attribute.cluster_membet_access_fail[Cluster_Member_Num] = 0;
            
            Cluster_Reply send_cluster_reply;//创建入簇回复
            send_cluster_reply.TYPE = REPLY;
            send_cluster_reply.RA = packet.cluster_reply.SA;
            send_cluster_reply.SA = attribute.MAC_ID;
            send_cluster_reply.CLUSTER_ID = attribute.Cluster_Member_Num;

            send_packet.cluster_rep = send_cluster_reply;
            send_packet.TYPE = REPLY;
            Send_Pack(send_packet);//发送入簇回复

            attribute.Cluster_Member_Num++;
        }
    }
}

static void Route_Reply_Handle(Packet *packet,Node_Attribute *attribute)//如果接收到的是入簇回复
{
    if(attribute.IF_ADD_CLUSTER == 0 && attribute.MAC_ID == packet.route_reply.RA)
    {
        //Send_Pack send_packet;
        attribute.IF_ADD_CLUSTER = 1;
        attribute.Cluster_Center = packet.Cluster_Reply.SA;
        attribute.Cluster_ID = packet.Cluster_Reply.CLUSTER_ID;
    }

}

static void Beacon_Handle(Packet *packet,Node_Attribute *attribute)//如果收到的是beacon帧
{
    if(attribute.CLUSTER_ID == packet.beacon.SA)
    {
        /*
        此处本该利用beacon帧中B-CFP、B-CAP、T-Slot、B-SLEEP 进行同步，但我不清楚如何时钟如何进行同步
        */

        uint8 ever_wait_time = SLOT;
        wait(ever_wait_time * attribute.MAC_ID)//等待每个时隙时间*簇内ID号，wait()函数未未定义
        Send_Pack send_packet;
        //发送数据帧
        Data_Pack data;
        data.SA = attribute.MAC_ID;
        data.RA = attribute.CLUSTER_ID;
        data.IF_C2C = 0;

        send_packet.data_pack = data;
        send_packet.TYPE = DATAPACK;
        Send_Pack(send_packet);
        uint8 beacon_time = TBEACON;
        uint8 sleep_time = beacon_time - (ever_wait_time * attribute.MAC_ID + ever_wait_time);//睡眠，直到下一个beacon帧来之前
        sleep(sleep_time);//sleep()函数未定义，设备睡眠一段时间后唤醒，开始监听
    }

    else if( attribute.IF_ADD_CLUSTER == 0 || attribute.IF_CENTER == MEMBER)//如果该节点未入簇，则接收到beacon后发送入簇请求
    {
        Send_Pack send_packet;
        Cluster_Request cluster_request;
        cluster_request.SA = attribute.MAC_ID;
        cluster_request.RA = packet.beacon.SA;
        send_packet.cluster_req = Cluster_Request;
        send_packet.TYPE = REQUEST;
        Send_Pack(send_packet);
        }
    }
}


static void Communicatin_Ack_Handle(Packet *packet,Node_Attribute *attribute)//如果收到的是数据确认帧
{
    if(attribute.MAC_ID == packet.c_ack.RA )
    {
        attribute.IF_GET_ACK = 1;
    }
}

static void Data_Pack_Handle(Packet *packet,Node_Attribute *attribute)//如果收到的是数据帧
{
    if(attribute.MAC_ID == packet.data_pack.RA )
    {
        if(packet.data_pack.IF_C2C == 0)//如果该数据帧是用来进行簇内通信,(本设备是簇头)
        {
            uint8 member_id;
            for(uint16 i = 0;i < attribute.Cluster_Member_Num;i++) //查找接收数据对应的簇序号
            {
                if(attribute.Cluster_Member[i] == packet.data_pack.SA)
                    member_id = i;
            }
            attribute.cluster_membet_access_fail[member_id] = 0;//簇内该节点接收失败的轮次清零
        }
    
        else //该数据帧上汇数据到簇首（本设备是路由节点 或服务器）
        {   
            if(attribute.IF_SERVER == SERVER)//如果本设备是服务器，则更新数据
            {
                for(uint16 i = 0;i < MAX_CLUSTER;i++)
                {
                    //扫描，如果节点不在服务器内，则将该节点地址添加到服务器中
                    uint8 IF_IN_SERVER = 0;
                    for(uint16 j = 0;i < attribute.ACC_Number;i++)
                    {
                        if(attribute.ACC_Node_ID[j] == packet.Data_Body[i]) IF_IN_SERVER = 1;
                    }

                    if(IF_IN_SERVER == 0 && attribute.ACC_Number < TOTAL_NODES)
                    {
                        attribute.ACC_Node_ID[attribute.ACC_Number] = packet.Data_Body[i];
                        attribute.ACC_Number++;
                    }
                }
            }

            else if(attribute.IF_ROUTE = ROUTE)//如果本设备是路由节点则向上转发数据帧
            {
                Send_Pack send_packet;
                //回复数据确认帧
                Communication_ACK com_ack;
                com_ack.RA =  packet.data_pack.SA;
                send_packet.c_ack = com_ack;
                send_packet.TYPE = COMMUNICATEACK;
                Send_Pack(send_packet);
                //向下一跳转发数据帧
                Data_Pack data_packet;
                data_packet.RA = attribute.Next_Route;
                data_packet.SA = attribute.MAC_ID;
                data_packet.Data_Body = packet.data_pack.Data_Body;
                send_packet.data_pack = data_packet;
                send_packet.TYPE = DATAPACK;    
                Send_Pack(send_packet);

                attribute.IF_GET_ACK = 0;
            }
        } 
    }    
}

static void ReceiveHandle(Packet *packet,Node_Attribute *attribute)//根据接收到帧的不同进行不同处理
{   
    switch(packet.TYPE)
    {
        case BROADCAST:
        Route_Broadcast_Handle(packet,attribute);
        break;

        case ROUTEACK:
        Route_Ack_Handle(packet,attribute);
        break;

        case REQUEST:
        Route_Request_Handle(packet,attribute);
        break;

        case REPLY:
        Route_Reply_Handle(packet,attribute);
        break;

        case BEACON:
        Beacon_Handle(packet,attribute);
        break;

        case COMMUNICATEACK:
        Communicatin_Ack_Handle(packet,attribute);
        break;

        case DATAPACK:
        Data_Pack_Handle(packet,attribute);
        break;

        default:
        break;
    }
}
//************************************************************************************************************

static void appMember(Node_Attribute *attribute)//节点为普通节点
{
    attribute.IF_SERVER = DEVICE; 

    //生成一个概率，使节点成为路由节点
    uint8 random_route = random(10);//此处用了默认的random，不知道行不行
    if(random_route < 2 )
    attribute.IF_ROUTE = ROUTE;
    halLcdWriteLine(HAL_LCD_LINE_1, "Member");
    halLcdWriteLine(HAL_LCD_LINE_2, "Ready");
#ifdef ASSY_EXP4618_CC2420
    halLcdClearLine(1);
    halLcdWriteSymbol(HAL_LCD_SYMBOL_RX, 1);
#endif

    // Initialize BasicRF
    basicRfConfig.myAddr = MEMBER_ADDR;
    if(basicRfInit(&basicRfConfig)==FAILED) {
      HAL_ASSERT(FALSE);
    }
    basicRfReceiveOn();
    //以上取自light_switch.c，未改动
    
    uint8 Random_Center = random(CREATE_CLUSTER_LISTEN_TIME);//产生一个在500内的随机数
    uint8 IF_GET_BEACON = 0;
    uint8 IF_MEMBER = 0;
    //设置一个定时器
    while(Set_Timer(Random_Center) && attribute.IF_ADD_CLUSTER == 0)//定时器，到Random_Center时间后返回TRUE,该函数还未定义，需要修改！！！！！！！！
    {
        //节点开始监听，需要补充！！！！！
            while(!basicRfPacketIsReady());

            /*
            此处用来读取缓存中接收到的数据
            随后将接收到的序列译为帧结构
            */
            RxData rxdata = GetInformation();//假设 Rxdata是二进制序列接收后的类型
            Send_Pack send_packet = Rx2Pack(RxData)
            
            if(send_packet.TYPE = beacon)
            {
                IF_GET_BEACON = 1;
            }
    

        if(attribute.IF_ADD_CLUSTER == 1)
        {
            IF_MEMBER = 1;
        }
    }

    if( IF_GET_BEACON == 1)//如果在期间收到beacon帧，则再监听500秒（等待入簇回复）
    {
        while(!Set_Timer(WAIT_ACK_TIME))
        {
            //节点开始监听，需要补充！！！！！！！
            while(!basicRfPacketIsReady());

            /*
            此处用来读取缓存中接收到的数据
            随后将接收到的序列译为帧结构
            */
            RxData rxdata = GetInformation();//假设 Rxdata是二进制序列接收后的类型
            Send_Pack send_packet = Rx2Pack(RxData)
            if(send_packet.TYPE = beacon)
            {
                IF_GET_BEACON = 1;
            }

        if(attribute.IF_ADD_CLUSTER == 1)
        {
            IF_MEMBER = 1;
        }
            if(attribute.IF_ADD_CLUSTER == 1)
            {
                IF_MEMBER = 1;
            }
        }
    }

    if(IF_MEMBER == 1)//本设备为普通节点，开始监听
    {
        while(!basicRfPacketIsReady());

            /*
            此处用来读取缓存中接收到的数据
            随后将接收到的序列译为帧结构
            */
            RxData rxdata = GetInformation();//假设 Rxdata是二进制序列接收后的类型
            Send_Pack send_pack = Rx2Pack(RxData)
            ReceiveHandle(send_pack,attribute)； //此函数用来根据接收到的分组类型采取措施
    }

    else//本设备为簇首
    {
        attribute.IF_CENTER == CENTER;
        appCenter(attribute);
    }

}

/***********************************************************************************

*/
static void appCenter(Node_Attribute *attribute)//节点为簇头
{
    halLcdWriteLine(HAL_LCD_LINE_1, "Center");
    halLcdWriteLine(HAL_LCD_LINE_2, "Joystick Push");
    halLcdWriteLine(HAL_LCD_LINE_3, "Send Command");
#ifdef ASSY_EXP4618_CC2420
    halLcdClearLine(1);
    halLcdWriteSymbol(HAL_LCD_SYMBOL_TX, 1);
#endif

    pTxData[0] = LIGHT_TOGGLE_CMD;

    // Initialize BasicRF
    basicRfConfig.myAddr = CENTER_ADDR;
    if(basicRfInit(&basicRfConfig)==FAILED) {
      HAL_ASSERT(FALSE);
    }

    // Keep Receiver off when not needed to save power
    basicRfReceiveOff();

    //以上内容为原文件内容
    uint8 ever_time = SLOT;
    uint8 TDMA_Time = ever_time * MAX_CLUSTER;
    uint8 CSMA_Time = 4 * TDMA_Time;

    while(TRUE)
    {
        //发送beacon帧
        Send_Pack send_packet;
        send_packet.beacon.SA = attribute.MAC_ID;
        send_packet.beacon.B_CFP = now_time();//now_time()表示当前时间，该函数需要补充定义或替换!!!
        send_packet.beacon.B_CAP = now_time() + TDMA_Time;
        send_packet.beacon.B_SLEEP = now_time() + TDMA_Time + CSMA_Time;
        for(uint16 i = 0;i < MAX_CLUSTER; i++)
        {
            if(i < attribute.Cluster_Member_Num)    
                send_packet.beacon.T_SLOT[i] = SLOT;
        
            else
                send_packet.beacon.T_SLOT[i] = 0;
        }
        send_packet.TYPE = BEACON;
        Send_Pack(send_packet);

        while(Set_Timer(TDMA_Time))//开始TDMA
        {
            while(!basicRfPacketIsReady());

            //开始侦听，需要补充！！！！！
            /*
            此处用来读取缓存中接收到的数据
            随后将接收到的序列译为帧结构
            */
            RxData rxdata = GetInformation();//假设 Rxdata是二进制序列接收后的类型
            Send_Pack send_pack = Rx2Pack(RxData)
            if(send_pack.TYPE == DATAPACK)//TDMA过程中只能接收数据帧
                ReceiveHandle(send_pack,attribute)； //此函数用来根据接收到的分组类型采取措施
        }
        //将簇内信息打包，发送数据帧
        send_packet.data_pack.SA = attribute.MAC_ID;
        send_packet.data_pack.RA = attribute.Next_Route;
        send_packet.data_pack.Data_Body[0] = attribute.MAC_ID;
        for(uint16 i = 1; i < attribute.Cluster_Member_Num + 1;i++)
        {
            send_packet.data_pack.Data_Body[i] = attribute.Cluster_Member[i];
        }
        send_packet.TYPE = DATAPACK;
        Send_Pack(send_packet);
        
        while(Set_Timer(CSMA_Time)) //开始竞争接入周期
        {
            while(!basicRfPacketIsReady());
            //开始侦听，需要补充！！！！！
            /*
            此处用来读取缓存中接收到的数据
            随后将接收到的序列译为帧结构
            */
            RxData rxdata = GetInformation();//假设 Rxdata是二进制序列接收后的类型
            Send_Pack send_pack = Rx2Pack(RxData)
            ReceiveHandle(send_pack,attribute)； //此函数用来根据接收到的分组类型采取措施 
        }
    }
}

/***********************************************************************************
*/

static void appServer(Node_Attribute *attribute)//节点为服务器
{
    halLcdWriteLine(HAL_LCD_LINE_1, "Server");
    halLcdWriteLine(HAL_LCD_LINE_2, "Ready");
#ifdef ASSY_EXP4618_CC2420
    halLcdClearLine(1);
    halLcdWriteSymbol(HAL_LCD_SYMBOL_RX, 1);
#endif

    // Initialize BasicRF
    basicRfConfig.myAddr = attribute.MAC_ADDR;
    if(basicRfInit(&basicRfConfig)==FAILED) {
      HAL_ASSERT(FALSE);
    }
    basicRfReceiveOn();
    //以上是原文件的代码

    attribute.IF_SERVER = SERVER;
    while(TRUE)
    {
            while(Set_Timer(START_ROUTE_TIME)) //在发送路由广播前保存监听状态
        {
            while(!basicRfPacketIsReady());
            //开始侦听，需要补充！！！！！
            /*
            此处用来读取缓存中接收到的数据
            随后将接收到的序列译为帧结构
            */
            RxData rxdata = GetInformation();//假设 Rxdata是二进制序列接收后的类型
            Send_Pack send_pack = Rx2Pack(RxData)
            ReceiveHandle(send_pack,attribute)； //此函数用来根据接收到的分组类型采取措施 
        }

        //监听一段时间后发送路由广播

        Send_Pack send_packet;
        send_packet.route_broadcast.SA = attribute.MAC_ID;
        send_packet.route_broadcast.DA = 0;//表示接收点是所有节点
        send_packet.TYPE = BROADCAST;
        Send_Pack(send_packet);
    }

}

/***********************************************************************************
* @fn          main
*
* @brief       
*
* @param       basicRfConfig - file scope variable. Basic RF configuration
*              data
*              appState - file scope variable. Holds application state
*
* @return      none
*/
void main(void)
{
    uint8 appMode = DEVICE;//角色初始化为普通设备

    // Config basicRF
    basicRfConfig.panId = DEFAULT_PAN_ID;//初始化节点都为同一个PAN_ID
    basicRfConfig.channel = RF_CHANNEL;// 都为同一个信道
    basicRfConfig.ackRequest = FALSE;// 不采用默认的ack，我们自己写

    // Initalise board peripherals
    halBoardInit();

    // Initalise hal_rf
    if(halRfInit()==FAILED) {
      HAL_ASSERT(FALSE);
    }

    // Indicate that device is powered
    halLedSet(1);

    halMcuWaitMs(350);
    halLcdClear();
    //前面是初始化内容，沿用light_switch.c 内容，不知道是不是要修改


    Node_Attribute *attribute;//节点属性初始化
    attribute.MAC_ID = MAC_ADDR;
    attribute.IF_ROUTE = NONE;
    attribute.IF_CENTER = MEMBER;
    attribute.IF_CLUSTER_FULL = 0;
    attribute.IF_HAVEROUTE = 0;
    attribute.IF_FRIST_ROUTE = 0;
    attribute.IF_ADD_CLUSTER = 0;

    // 设置节点角色(服务器/普通节点)DEVICE|SERVER
    appMode = DEVICE;

    // Transmitter application
    if(appMode == DEVICE) {
        appMember(attribute);
    }
    else if(appMode == SERVER) {
        // No return from here
        appServer(attribute);
    }
    // Role is undefined. This code should not be reached
    HAL_ASSERT(FALSE);
}


/****************************************************************************************

***********************************************************************************/
