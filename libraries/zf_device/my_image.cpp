#include "zf_common_headfile.hpp"
#include "my_image.hpp"
#include "my_imu.hpp"
#include "motor_ctrl.hpp"
#include <cmath>

int8  ncnn_ctrl_flag=0;
uint8 image_two_value[UVC_H][UVC_W];
 uint8_t* gray_image = new uint8_t[320 * 240];

int threshold;
int Longest_White_Column_Left[2];//0是长度，1是第几列
int Longest_White_Column_Right[2];
int Search_Stop_Line;//搜索截止行,只记录长度，想要坐标需要用视野高度减去该值
int right_border;
volatile int Right_Lost_Flag[UVC_H];//右丢线数组
int left_border ;
volatile int Left_Lost_Flag[UVC_H];//左丢线数组，初始是0，不丢边是1，丢边是2
int8 Left_Line [UVC_H];       //左边线线数
int8 Right_Line[UVC_H];
volatile int White_Column[UVC_W];
volatile int Right_Lost_Time;    //边界丢线数
volatile int Left_Lost_Time;
volatile int Boundry_Start_Left;//第一个非丢线点,常规边界始起点
volatile int Boundry_Start_Right;//搜索截止行,只记录长度，想要坐标需要用视野高度减去该值
volatile int Both_Lost_Time;//两边丢线数
int Road_Wide[UVC_H];//实际赛宽数组
int8 Mid_Line[UVC_H];  //中线数组
volatile int right_line_lost;
volatile int left_line_lost;

//  const uint8 Weight[60]=//误差权重数组
//  {       0,0,0,0,0,0,0,0,0,0,
//          0,0,0,0,0,0,0,0,0,0,
//          0,0,0,0,0,
//          10,15,20,15,10,0,0,
//          0,0,0,0,0,
//          0,0,0,
//          0,0,0,0,0,0,0,0,0,0,
//          0,0,0,0,0,0,0,0,0,0,
         
         
         
         
//         };

    
         
         
         
         
         
       
// 拐点识别相关变量
uint8 Right_Up_Point_finish_flag = 0;
uint8 Left_Up_Point_finish_flag = 0;
uint8 Right_Down_Point_finish_flag = 0;
uint8 Left_Down_Point_finish_flag = 0;
uint8 Last_Right_Down_Point_finish_flag = 0;
uint8 Last_Left_Down_Point_finish_flag = 0;
uint8 Right_Down_Point[2] = {0};  // 0:x坐标(列), 1:y坐标(行)
uint8 Right_Up_Point[2] = {0};                        
uint8 Left_Down_Point[2] = {0};
uint8 Left_Up_Point[2] = {0};
uint8 Last_Left_Down_Point[2];
uint8 Last_Right_Down_Point[2]; 
uint8 Last_Right_Point[2];  // 角点历史值
uint8 Last_Left_Point[2];
uint8 findrcount = 0;
uint8 findlcount = 0;
uint8 Standard_Road_Wide[UVC_H]=//标准赛宽,这里推荐使用debug，写一个循环，在调试模式下打断点，直接读取这个数组值
{ 
8,9,10,12,13,14,16,17,18,20,
21,22,23,25,26,27,28,29,31,33,
33,35,36,37,39,40,41,42,44,45,
46,48,48,50,52,52,54,55,56,58,
59,60,61,63,64,65,67,67,69,70,
71,73,73,74,79,79,79,79,79,79,

};
//**********角点相关变量**********//
int8 left_down_guai[2];
//int8 left_up_guai[2];
int8 right_down_guai[2];
//int8 right_up_guai[2];
int8 Right_Down_Find=0;
int8 Left_Down_Find=0;
int8 Left_Up_Find=0;
int8 Right_Up_Find=0;
int8_t  Cross_Flag=0;
int8_t Island_State=0;
int8_t banmaxianFlag=0;
int8_t Chujie_Flag=0;
int changepoint[UVC_H];
int changepointhang[UVC_H];
// 方向数组定义为常量
static const int8 directionrd[5][2] = {{-1,1}, {-1,0}, {-1,-1}, {0,1}, {1,1}};
static const int8 directionld[5][2] = {{1,1}, {1,0}, {1,-1}, {0,-1}, {-1,-1}};
static const int8 directionru[5][2] = {{1,1}, {0,1}, {-1,1}, {-1,0}, {-1,-1}};
static const int8 directionlu[5][2] = {{-1,1}, {0,1}, {1,1}, {1,0}, {1,-1}};



/*------环岛变量------*/

int8  zuoxia;
int8  youxia;
int8  zuoup;
int8  youup;				
//uint8 Island_State=0;//环岛状态标志
     
 int8 Left_Island_Flag=0; //左右环岛标志
 int8 Right_Island_Flag=0;//左右环岛标志
 int8 Left_Up_Guai[2];    //四个拐点的坐标存储，[0]存y，第某行，{1}存x，第某列
 int8 Right_Up_Guai[2];   //四个拐点的坐标存储，[0]存y，第某行，{1}存x，第某列
 int8 left_down_line;
 int8 left_up_line;
 int8 right_down_line;
 int8 right_up_line;
 int8 monotonicity_change_line[2]={0};//单调性改变点坐标，[0]寸某行，[1]寸某列				
				
	int8 zuohuandaoFlag=0;
int8 DownLeftFlag;
int8 DownRightFlag;
int8 UpLeftFlag;
int8 UpRightFlag;
int8 DownLeft[2];
int8 DownRight[2];
int8 UpLeft[2];
int8 UpRight[2];//0是y，1是x			
 


/*------环岛变量------*/






     
/*-------------------------------------------------------------------------------------------------------------------
  @brief     快速大津求阈值，来自山威
  @param     image       图像数组
             col         列 ，宽度
             row         行，长度
  @return    null
  Sample     threshold=my_adapt_threshold(mt9v03x_image[0],MT9V03X_W, MT9V03X_H);//山威快速大津
  @note      据说比传统大津法快一点，实测使用效果差不多
-------------------------------------------------------------------------------------------------------------------*/
int my_adapt_threshold(uint8 *image, uint16 col, uint16 row)   //注意计算阈值的一定要是原图像
{
    #define GrayScale 256
    uint16 width = col;
    uint16 height = row;
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j;
    int pixelSum = width * height/4;
    int threshold = 0;
    uint8* data = image;  //指向像素数据的指针
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }
    uint32 gray_sum=0;
    //统计灰度级中每个像素在整幅图像中的个数
    for (i = 0; i < height; i+=2)
    {
        for (j = 0; j < width; j+=2)
        {
            pixelCount[(int)data[i * width + j]]++;  //将当前的点的像素值作为计数数组的下标
            gray_sum+=(int)data[i * width + j]; 
                  //灰度值总和
        }
    }
    //计算每个像素值的点在整幅图像中的比例
    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
    }
    //遍历灰度级[0,255]
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
    for (j = 0; j < GrayScale; j++)
    {
        w0 += pixelPro[j];  //背景部分每个灰度值的像素点所占比例之和   即背景部分的比例
        u0tmp += j * pixelPro[j];  //背景部分 每个灰度值的点的比例 *灰度值
        w1=1-w0;
        u1tmp=gray_sum/pixelSum-u0tmp;
        u0 = u0tmp / w0;              //背景平均灰度
        u1 = u1tmp / w1;              //前景平均灰度
        u = u0tmp + u1tmp;            //全局平均灰度
        deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;
            threshold = j;
        }
        if (deltaTmp < deltaMax)
        {
            break;
        }
    }
    if(threshold<=100)threshold=100;
    return threshold;
}


void Image_Binarization(int threshold)
{
    uint16 i, j;
    for (i = 0; i < UVC_HEIGHT; i+=4)
    {
        for (j = 0; j < UVC_WIDTH; j+=4)
        {
            if (gray_image[i * UVC_WIDTH + j] >= threshold)
                image_two_value[i/4][j/4] = 255;  // 白
            else
                image_two_value[i/4][j/4] = 0;    // 黑
        }
    }
}

void Image_Binarization2(int threshold)//图像二值化
{
    uint16 i,j; 
    for(i=30;i<=UVC_H-30-1;i++)
    {
        for(j=0;j<UVC_W;j++)//灰度图的数据只做判断，不进行更改，二值化图像放在了新数组中
        {
            if(gray_image[i*UVC_W+j]>=threshold)
                image_two_value[60-(i-29)][160-j-1]=255;//白
            else
                {image_two_value[60-(i-29)][160-j-1]=0;}//黑                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
              
        }
    }
    
}





/*-------------------------------------------------------------------------------------------------------------------
  @brief     双最长白列巡线
  @param     null
  @return    null
  Sample     Longest_White_Column_Left();
  @note      最长白列巡线，寻找初始边界，丢线，最长白列等基础元素，后续读取这些变量来进行赛道识别
-------------------------------------------------------------------------------------------------------------------*/
void Longest_White_Column(uint8 Island_Flag_1)//最长白列巡线
{
    int i, j;
    uint8 start_column=15;//最长白列的搜索区间
    uint8 end_column=64;
    int16 stop_vel=0;
    int left_border = 0, right_border = 0;//临时存储赛道位置
    Longest_White_Column_Left[0] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Left[1] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Right[0] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Right[1] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Right_Lost_Time = 0;    //边界丢线数
    Left_Lost_Time  = 0;
    Boundry_Start_Left  = 0;//第一个非丢线点,常规边界起始点
    Boundry_Start_Right = 0;
    Both_Lost_Time = 0;//两边同时丢线数
        if(Island_Flag_1==0)
		{
			start_column=15;
		 end_column=64;//正常
	
		}

		if(Island_Flag_1==1)
		{
		end_column=55;//左环岛
	
		}

		
		if(Island_Flag_1==2)
		{
			start_column=24;//右环岛
			
		}
        if(ncnn_ctrl_flag==1)
        {
            start_column=0;
        }
        if(ncnn_ctrl_flag==3)
        {
            end_column=79;
        }
    for (i = 0; i <60; i++)//数据清零
    {
        Right_Lost_Flag[i] = 0;
        Left_Lost_Flag[i] = 0;
        Left_Line[i] = 0;
        Right_Line[i] =UVC_W-1;
        Mid_Line[i]=40;
    }
     for(i=0;i<UVC_W;i++)
    {
        White_Column[i] = 0;
    } 
 
 //从左到右，从下往上，遍历全图记录范围内的每一列白点数量
    for (j =start_column; j<=end_column; j++)
{   
    
    for (i = UVC_H - 1; i >= 0; i--)
    {   
        if(image_two_value[i][j] == IMG_BLACK)
            break;
        else
            White_Column[j]++;
    }
   // printf("%d\r\n",White_Column[80]);
}
Longest_White_Column_Left[0] =0;

for(i=start_column;i<=end_column;i++)
{
    if (Longest_White_Column_Left[0] < White_Column[i])//找最长的那一列
    {
        Longest_White_Column_Left[0] = White_Column[i];//【0】是白列长度
        Longest_White_Column_Left[1] = i;              //【1】是下标，第j列
    }
}
//从右到左找右左边最长白列
Longest_White_Column_Right[0] = 0;//【0】是白列长度
for(i=end_column;i>=start_column;i--)//从右往左，注意条件，找到左边最长白列位置就可以停了
{
    if (Longest_White_Column_Right[0] < White_Column[i])//找最长的那一列
    {
        Longest_White_Column_Right[0] = White_Column[i];//【0】是白列长度
        Longest_White_Column_Right[1] = i;              //【1】是下标，第j列
    }
}
    Search_Stop_Line = Longest_White_Column_Left[0];
    /* Longest_White_Column_Left[0] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Left[1] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Right[0] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列
    Longest_White_Column_Right[1] = 0;//最长白列,[0]是最长白列的长度，[1】是第某列*/
   
    //printf("%d\r\n",stop_vel);

    //printf("%d\r\n",stop_vel);
    for (i = UVC_H - 1; i >=UVC_H-Search_Stop_Line; i--)//常规巡线
    {//从最下面一行，访问到我的有效是视野行
        for (j = Longest_White_Column_Right[1]; j <= UVC_W - 1 - 2; j++)
        {   
            if (image_two_value[i][j] ==IMG_WHITE && image_two_value[i][j + 1] == IMG_BLACK && image_two_value[i][j + 2] == IMG_BLACK)//白黑黑，找到右边界
            {
                right_border = j+1;
                Right_Lost_Flag[i] = 1; //右丢线数组，初始置0，不丢线置1，丢线置2
                break;
            }
            else if(j>=UVC_W-1-2)//没找到右边界，把屏幕最右赋值给右边界
            {
                right_border = j+1;
                Right_Lost_Flag[i] = 2; //右丢线数组，初始置0，不丢线置1，丢线置2
                break;
            }


        }
        for (j = Longest_White_Column_Left[1]; j >= 0 + 2; j--)//往左边扫描
        {//从最下面一行，访问到我的有效是视野行
            if (image_two_value[i][j] ==IMG_WHITE && image_two_value[i][j - 1] == IMG_BLACK && image_two_value[i][j - 2] == IMG_BLACK)//黑黑白认为到达左边界
            {
                left_border = j-1;
                Left_Lost_Flag[i] = 1; //左丢线数组，初始置0，不丢线置1，丢线置2
                break;
            }
            else if(j<=0+2)
            {
                left_border = j-1;//找到头都没找到边，就把屏幕最左右当做边界
                Left_Lost_Flag[i] = 2; //左丢线数组，初始置0，不丢线置1，丢线置2
                break;
            }


        }
        Left_Line [i] = left_border;       //左边线线数组
        Right_Line[i] = right_border;       //右边线线数组
     
       //printf("%d\r\n",Right_Line[i]);
            
    }
    
    //printf("start\r\n");
      for (i = UVC_H-1; i>= 0; i--)//赛道数据初步分析
    {
        if (Left_Lost_Flag[i]  == 2)//单边丢线数
            Left_Lost_Time++;
        if (Right_Lost_Flag[i] == 2)
            Right_Lost_Time++;
        if (Left_Lost_Flag[i] == 2 && Right_Lost_Flag[i] == 2)//双边丢线数
            Both_Lost_Time++;
        if (Boundry_Start_Left ==  0 && Left_Lost_Flag[i]==1)//记录第一个非丢线点，边界起始点
            Boundry_Start_Left = i;
        if (Boundry_Start_Right == 0 && Right_Lost_Flag[i] == 1)
            Boundry_Start_Right = i;
        Road_Wide[i]=Right_Line[i]-Left_Line[i];
       
       /*  if(Right_Lost_Flag[i]==2)Right_Line[i]=Right_Line[UVC_H-Search_Stop_Line];
        if(Left_Lost_Flag[i]==2)Left_Line[i]=Left_Line[UVC_H-Search_Stop_Line];  */
       
       

        //if(Left_Lost_Flag[i]==0)Road_Wide[i]= (160+Longest_White_Column_Right[1])/2;++h+hbbn+jmkkkkkkkkkkkkkkkkkkkk
      // printf(" %d\r\n", Road_Wide[i]);
    }   // printf("%d\r\n",Left_Lost_Time);
     
   //printf("end\r\n");
   

}
/********十字********/

/*-------------------------------------------------------------------------------------------------------------------
  @brief      右赛道连续性检测
  @param      start - 起始行, end - 终止行
  @return     连续返回0，不连续返回断线行数
  @note      连续性的阈值设置为5
-------------------------------------------------------------------------------------------------------------------*/
int8 Continuity_Change_Right(int8 start, int8 end)
{
    int8 i, t, continuity_change_flag = 0;
    
    if (Right_Lost_Time >= 81)  // 0.9 * 60=54
        return 1;
        
    if (start >= 85)  // 60-5=55
        start = 85;
        
    if (end <= 5)
        end = 5;
        
    if (start < end)  // 互换
    {
        t = start;
        start = end;
        end = t;
    }
 
    for (i = start; i >= end; i--)
    {
        if (abs(Right_Line[i] - Right_Line[i-1]) >= 5)  // 连续性阈值
        {
            continuity_change_flag = i;
            break;
        }
    }
    
    return continuity_change_flag;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief      左赛道连续性检测
  @param      start - 起始行, end - 终止行
  @return     连续返回0，不连续返回断线行数
  @note      连续性的阈值设置为5
-------------------------------------------------------------------------------------------------------------------*/
int8 Continuity_Change_Left(int8 start, int8 end)
{
    int8 i, t, continuity_change_flag = 0;
    
    if (Left_Lost_Time >= 54)  // 0.9 * 60=54
        return 1;
        
    if (Search_Stop_Line <= 5)
        return 1;
        
    if (start >= 85)  // 60-5=55
        start = 85;
        
    if (end <= 5)
        end = 5;
        
    if (start < end)  // 互换
    {
        t = start;
        start = end;
        end = t;
    }

    for (i = start; i >= end; i--)
    {
        if (abs(Left_Line[i] - Left_Line[i-1]) >= 5)  // 连续性阈值
        {
            continuity_change_flag = i;
            break;
        }
    }
    
    return continuity_change_flag;
}


/**
* @brief 最小二乘法
* @param uint8 begin                输入起点
* @param uint8 end                  输入终点
* @param uint8 *border              输入需要计算斜率的边界首地址
*  @see CTest       Slope_Calculate(start, end, border);//斜率
* @return 返回说明
*     -<em>false</em> fail
*     -<em>true</em> succeed
*/
float Slope_Calculate(uint8 begin, uint8 end, int8 *border)
{
    float xsum = 0, ysum = 0, xysum = 0, x2sum = 0;
    int16 i = 0;
    float result = 0;
    static float resultlast;

    for (i = begin; i < end; i++)
    {
        xsum += i;
        ysum += border[i];
        xysum += i * (border[i]);
        x2sum += i * i;

    }
    if ((end - begin)*x2sum - xsum * xsum) //判断除数是否为零
    {
        result = ((end - begin)*xysum - xsum * ysum) / ((end - begin)*x2sum - xsum * xsum);
        resultlast = result;
    }
    else
    {
        result = resultlast;
    }
    return result;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      限幅函数
//  @param      限幅目标
//  @param      限幅下限 
//  @param      限幅上限
//  @return     value
//  @note       懂的都懂
//  Left_Black_Point[y]为我的左边线数组 Right_Black_Point[y]为我的右边线数组
//-------------------------------------------------------------------------------------------------------------------
float limit_a_b(float value, float a, float b) 
{
    if (value < a) return a;      // 低于下限 → 返回a
    else if (value > b) return b; // 超过上限 → 返回b
    else return value;            // 正常范围 → 返回原值
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      拐点寻找
//  @param      findlcount   拐点距离丢线的行数，用于判断大小圆环，和区分P字和圆环
//  @param
//  @param
//  @param
//  @return     void
//  @note       采用5邻域的原理寻找拐点，下拐点从图像低端往上扫，上拐点从图像上方向下扫，左右扫线会在斑马线出现问题，
//-------------------------------------------------------------------------------------------------------------------
void Identify(void)
{
    uint8 findr_x = 0;    //右点
    uint8 findr_y = 0;
    uint8 examr_x = 0;
    uint8 examr_y = 0;
    uint8 findl_x = 0;    //左点
    uint8 findl_y = 0;
    uint8 examl_x = 0;
    uint8 examl_y = 0;
    uint8 star = 0;
    uint8 end = 0;
    uint8 examcount = 0;
    //uint8 count;
    //uint8 examerror;
//    uint8 dircount;
    // int8 directionrd[5][2] =  {{-1,1}, {-1,0}, {-1,-1}, {0,1}, {1,1}};  //顺时针下方向数组先x再y
    // int8 directionld[5][2] =  {{1,1}, {1,0}, {1,-1}, {0,-1}, {-1,-1}};  //逆时针下方向数组
    // int8 directionru[5][2] =  {{1,1}, {0,1}, {-1,1}, {-1,0}, {-1,-1}};  //逆时针上方向数组
    // int8 directionlu[5][2] =  {{-1,1}, {0,1}, {1,1}, {1,0}, {1,-1}};  //逆时针上方向数组
 
    //每次采集后都对拐点标志位清零
    if(Right_Up_Point_finish_flag == 1)
        Right_Up_Point_finish_flag = 0;
    if(Left_Up_Point_finish_flag == 1)
        Left_Up_Point_finish_flag = 0;
    if(Right_Down_Point_finish_flag == 1)
        Right_Down_Point_finish_flag = 0;
    if(Left_Down_Point_finish_flag == 1)
        Left_Down_Point_finish_flag = 0;
    if(Last_Right_Down_Point_finish_flag == 1)
    Last_Right_Down_Point_finish_flag = 0;
    if(Last_Left_Down_Point_finish_flag == 1)
    Last_Left_Down_Point_finish_flag = 0;    
    
    if(Island_State==0&&Continuity_Change_Right(55,10)!=0&&Continuity_Change_Left(55,10)!=0)
   {
        for(uint8 y = 55 ; y >= 15 ; y--)
    {
        if(Right_Down_Point_finish_flag == 0)
        {
            if(y > UVC_H-Search_Stop_Line && Right_Lost_Flag[y-1]==2 && Right_Lost_Flag[y-2]==2 && image_two_value[y][Right_Line[y]-6] ==IMG_WHITE
               && image_two_value[y-2][Right_Line[y]] == IMG_WHITE&& image_two_value[y-5][Right_Line[y]]== IMG_WHITE)    //右下拐点&& y > Endline 
            {
                star=y;
                for(uint8 y=star;y<=(star+18);y++)//此处参数需要确定
                {
                    if(Right_Line[y]<75 &&abs(Right_Line[y+1]-Right_Line[y])<3
                    && abs(Right_Line[y+2]-Right_Line[y])<4)//此处参数需要确定这个75有可能需要改
                    {
                        findr_x=Right_Line[y];
                        findr_y=y;
                        for(uint8 dircount = 0;dircount<5;dircount++)
                        {
                            examr_x=findr_x+directionrd[dircount][0];
                            examr_y=findr_y+directionrd[dircount][1];
                            if(image_two_value[examr_y][examr_x]==255)
                            {
                                examcount++;
                            }
                        }
                        if(examcount >= 4)//这里可以改成5,会更准确但是有可能太严格
                        {
                            examcount=0;
                            Right_Down_Point[0]=findr_x;
                            Right_Down_Point[1]=findr_y;
                            if(abs( Last_Right_Down_Point[1]-Right_Down_Point[1])<=5)
                            {
                                Last_Right_Down_Point_finish_flag = 1;
                            }
                            else
                                Last_Right_Down_Point_finish_flag = 0;

                            Last_Right_Down_Point[0]=Right_Down_Point[0];
                            Last_Right_Down_Point[1]=Right_Down_Point[1];
                            
                           Right_Down_Point_finish_flag = 1;
//                            if(Last_Right_Point[0] == 0)
//                            {
//                                Last_Right_Point[0] = Right_Down_Point[1];
//                            }
//                            if(Emo_Uint8_dec(Right_Down_Point[1],Last_Right_Point[0]) < -15)
//                            {
//                                Right_Down_Point_finish_flag = 0;
//                            }
//                            else
//                            {
                                
//                                Last_Right_Point[0] = Right_Down_Point[1];
//                            }
    //                        findrcount = (int)y-(int)star;
                            break;
                        }
                        else
                        {
                            Right_Down_Point_finish_flag = 0;
                            examcount=0;
                        }
                    }
                    if(y==55)
                    {
                        Right_Down_Point_finish_flag=0;
                    }
                }
            }
        }
        if(Left_Down_Point_finish_flag == 0)
        {
            if(y > UVC_H-Search_Stop_Line && Left_Lost_Flag[y-1]==2&& Left_Lost_Flag[y-2]==2 && image_two_value[y][Left_Line[y]+6] ==IMG_WHITE
                &&image_two_value[y-2][Left_Line[y]] == IMG_WHITE&& image_two_value[y-5][Left_Line[y]] == IMG_WHITE)     //左下拐点&& y > Endline
            {
                star=y;
                for(uint8 y=star;y<=(star+18);y++)
                {
                    if(Left_Line[y]>5 &&abs(Left_Line[y+1]-Left_Line[y])<3
                       && abs(Left_Line[y+2]-Left_Line[y])<4)
                    {
                        findl_x=Left_Line[y];
                        findl_y=y;
                        for(uint8 dircount = 0;dircount<5;dircount++)
                        {
                            examl_x=findl_x+directionld[dircount][0];
                            examl_y=findl_y+directionld[dircount][1];
                            if(image_two_value[examl_y][examl_x]==255)
                            {
                                examcount++;
                            }
                        }
                        if(examcount>=4 )
                        {
                            examcount=0;
                            Left_Down_Point[0]=findl_x;
                            Left_Down_Point[1]=findl_y;

                            if(abs( Last_Left_Down_Point[1]-Left_Down_Point[1])<=5)
                           {
                               Last_Left_Down_Point_finish_flag = 1;
                           }
                           else
                               Last_Left_Down_Point_finish_flag = 0;

                           Last_Left_Down_Point[0]=Left_Down_Point[0];
                           Last_Left_Down_Point[1]=Left_Down_Point[1];
//                            if(Last_Left_Point[0]  == 0)
//                            {
//                                Last_Left_Point[0] = Left_Down_Point[1];
//                            }
//                            if(Emo_Uint8_dec(Left_Down_Point[1],Last_Left_Point[0]) < -15)
//                            {
//                                Left_Down_Point_finish_flag = 0;
//                            }
//                            else
//                            {
                                Left_Down_Point_finish_flag = 1;
//                                Last_Left_Point[0] = Left_Down_Point[1];
//                            }
    //                        findlcount = y-star;
                            break;
                        }
                        else
                        {
                            Left_Down_Point_finish_flag = 0;
                            examcount=0;
                        }
                    }
                    if(y==55)
                    {
                        Left_Down_Point_finish_flag=0;
                    }
                }
            }
        }
//        if(CrossLeft_Down_Point_finish_flag==1 && CrossRight_Down_Point_finish_flag==1)
//        {
//            break;
//        }
    }
    if(Left_Down_Point_finish_flag==1 && Right_Down_Point_finish_flag==1)
    end=Right_Down_Point[1];
else if(Left_Down_Point_finish_flag==1)
    end=Left_Down_Point[1];
else if(Right_Down_Point_finish_flag==1)
    end=Right_Down_Point[1];
else
    end=40;//参数需要再次确定
    for(uint8 y=8;y<=end;y++)//此处也是
    {
        if(Right_Up_Point_finish_flag == 0)
        {
            if(y > UVC_H-Search_Stop_Line && Right_Lost_Flag[y+1]==2 && Right_Lost_Flag[y+2]==2 && Right_Lost_Flag[y+3]==2)   //右上拐点
            {
               star=y; 
               for(uint8 y=star;y>=(star-22);y--)
               {
                   if(Right_Line[y]<75 && abs(Right_Line[y-1]-Right_Line[y])<4
                   && abs(Right_Line[y-2]-Right_Line[y])<4 && image_two_value[y][Right_Line[y]-6] == IMG_WHITE
                   && image_two_value[y-1][Right_Line[y-1]-5] == IMG_WHITE&& image_two_value[y-2][Right_Line[y]-5] == IMG_WHITE
                   && image_two_value[y-3][Right_Line[y]-5] == IMG_WHITE& Right_Line[y] > Mid_Line[y] && image_two_value[y+3][Right_Line[y]] == IMG_WHITE
                   && image_two_value[y+5][Right_Line[y]] == IMG_WHITE && image_two_value[y+7][Right_Line[y]] == IMG_WHITE)//Right_Line[y]<155该参数需要确定，所有判断连续性的阈值都需要二次确定
                   {// Right_Black_Point[y] > Middle_Black_Point[y]右边界大于中线？此处存疑
                       findr_x=Right_Line[y];
                       findr_y=y;
                       for(uint8 dircount = 0;dircount<5;dircount++)
                       {
                           examr_x=findr_x+directionru[dircount][0];
                           examr_y=findr_y+directionru[dircount][1];
                           if(image_two_value[examr_y][examr_x]==255)                                                                                                                 
                           {
                               examcount++;
                           }
                       }
                       if(examcount>=4 && findr_y >UVC_H-Search_Stop_Line)
                       {
                           examcount=0;
                           Right_Up_Point[0]=findr_x;
                           Right_Up_Point[1]=findr_y;
                           if(Last_Right_Point[1] == 0)
                           {
                               Last_Right_Point[1] = Right_Up_Point[1];
                           }
                           if(Right_Up_Point[1] <8)//此值需要确定
                           {
                               Right_Up_Point[1] = Last_Right_Point[1];
                               Right_Up_Point_finish_flag = 0;
                           }
                           else
                           {
                               Right_Up_Point_finish_flag = 1;
                               Last_Right_Point[1] = Right_Up_Point[1];
                           }
                           findrcount=(int)star-(int)findr_y;
                           break;
                       }
                       else
                       {
                           Right_Up_Point_finish_flag = 0;
                           examcount=0;
                       }
                   }
                   if(y==5)//此处也是
                   {
                       Right_Up_Point_finish_flag=0;
                       break;
                   }
                   if(y == 1)
                   {
                       break;
                   }
               }
            }
        }
        if(Left_Up_Point_finish_flag == 0)
        {
            if(y > UVC_H-Search_Stop_Line && Left_Lost_Flag[y+1]==2 && Left_Lost_Flag[y+2]==2 && Left_Lost_Flag[y+3]==2)     //左上拐点
            {
                star=y;
                for(uint8 y=star;y>=(star-22);y--)
                {
                    if(Left_Line[y]>8 && abs(Left_Line[y-1]-Left_Line[y])<4
                   && abs(Left_Line[y-2]-Left_Line[y])<4 && image_two_value[y][Left_Line[y]+6] ==IMG_WHITE
                   && image_two_value[y-1][Left_Line[y-1]+5] == IMG_WHITE& image_two_value[y-2][Left_Line[y]+5] == IMG_WHITE
                   && image_two_value[y-3][Left_Line[y]+5] == IMG_WHITE && Left_Line[y] < Mid_Line[y] &&image_two_value[y+3][Left_Line[y]] == IMG_WHITE
                   && image_two_value[y+5][Left_Line[y]] == IMG_WHITE&& image_two_value[y+7][Left_Line[y]] == IMG_WHITE)
                    {
                        findl_x=Left_Line[y];
                        findl_y=y;
                        for(uint8 dircount = 0;dircount<5;dircount++)
                        {
                            examl_x=findl_x+directionlu[dircount][0];
                            examl_y=findl_y+directionlu[dircount][1];
                            if(image_two_value[examl_y][examl_x]==255)
                            {
                                examcount++;
                            }
                        }
                        if(examcount>=4 && findl_y >  UVC_H-Search_Stop_Line)
                        {
                            examcount=0;
                            Left_Up_Point[0]=findl_x;
                            Left_Up_Point[1]=findl_y;
                            if(Last_Left_Point[1] == 0)
                            {
                                Last_Left_Point[1] = Left_Up_Point[1];
                            }
                            if(Left_Up_Point[1] < 8)
                            {
                                Left_Up_Point[1] = Last_Left_Point[1];
                                Left_Up_Point_finish_flag = 0;
                            }
                            else
                            {
                                Left_Up_Point_finish_flag = 1;
                                Last_Left_Point[1] = Left_Up_Point[1];
                            }
                            findlcount=(int)star-(int)y;
                            break;
                        }
                        else
                        {
                            Left_Up_Point_finish_flag = 0;
                            examcount=0;
                        }
                    }
                    if(y==5)
                    {
                        Left_Up_Point_finish_flag=0;
                        break;
                    }
                    if(y == 1)
                    {
                        break;
                    }
                }
            }
        }
    }
}
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      十字补线函数
//  @param      无
//  @return     void
//  @note       懂的都懂
//-------------------------------------------------------------------------------------------------------------------
void shizibuxian(void)
{
    float k=0;  //斜率
    float b=0;  //截距
    float nihexielv_r;
    float nihexielv_l;
    float nihexielv_up_r;
    float nihexielv_up_l;
  
    if(Island_State==0&&LeftzhuantouFlag==0&&RightzhuantouFlag==0)
    {
       /*  continuity_change_left_flag_shizi=Continuity_Change_Left(UVC_H-1-5,5);//连续性判断
        continuity_change_right_flag_shizi=Continuity_Change_Right(UVC_H-1-5,5); */
    if((Both_Lost_Time>=10)&&(Both_Lost_Time<=45&&Continuity_Change_Right(50,15)!=0&&Continuity_Change_Left(50,15)!=0))//Both_Lost_Time>=3&&
    {//Right_Lost_Time>=10||Left_Lost_Time>=10||
        Cross_Flag=1;
    }
    else 
    {
        Cross_Flag=0;
    }
    if(Cross_Flag==1)
    {    /***********************************同时找到下拐点*****************************************************/
    if((Right_Down_Point_finish_flag==1&&Right_Up_Point_finish_flag==0&&Left_Down_Point_finish_flag==1&&Left_Up_Point_finish_flag==0)
            ||(Right_Down_Point_finish_flag==1&&Right_Up_Point_finish_flag==1&&Left_Down_Point_finish_flag==1&&Left_Up_Point_finish_flag==0)
            ||(Right_Down_Point_finish_flag==1&&Right_Up_Point_finish_flag==0&&Left_Down_Point_finish_flag==1&&Left_Up_Point_finish_flag==1))
    {   nihexielv_r=Slope_Calculate(Right_Down_Point[1],59,Right_Line);//1 0   0 1     0 0
        if(nihexielv_r!=0)                                             //1 1   1 1     1 1
        {
             k=(float)((float)1/nihexielv_r);

                 b=(float)Right_Down_Point[1] - ((float)Right_Down_Point[0]*k);
                 for(uint8 y=0;y<=Right_Down_Point[1];y++)
                    {
                     Right_Line[y]=(uint8)(((float)y-b)/k);  //两点之间补线
                     Right_Line[y]=limit_a_b(Right_Line[y],0,79);
                    }
        }

        nihexielv_l=Slope_Calculate(Left_Down_Point[1],59,Left_Line);
        if(nihexielv_l!=0)
        {
                k=(float)((float)1/nihexielv_l);
                    b=(float)Left_Down_Point[1] - ((float)Left_Down_Point[0]*k);
                    for(uint8 y=0;y<=Left_Down_Point[1];y++)
                       {
                        Left_Line[y]=(uint8)(((float)y-b)/k);  //两点之间补线
                        Left_Line[y]=limit_a_b(Left_Line[y],0,79);
                       }

        }
       /*  for(int i=0;i<=59;i++)
        {
            Mid_Line[i] = (Left_Line[i] + Right_Line[i]) >> 1;
        } */
    }
    /***********************************同时找到下拐点*****************************************************/
    //Padding_LineR(Right_Up_Point[0],Right_Up_Point[1],Right_Down_Point[0],Right_Down_Point[1]);
    //Padding_LineL(Left_Up_Point[0],Left_Up_Point[1],Left_Down_Point[0],Left_Down_Point[1]);
    /***********************************同时找到上下拐点*****************************************************/
    else if(Right_Down_Point_finish_flag==1&&Right_Up_Point_finish_flag==1&&Left_Down_Point_finish_flag==1&&Left_Up_Point_finish_flag==1
            &&Right_Down_Point[1]>Right_Up_Point[1]+3&&Left_Down_Point[1]>Left_Up_Point[1]+3)//此处需要测定
    {                                                                                        //1  1
        if((float)Right_Down_Point[0] - (float)Right_Up_Point[0]!=0)                         //1  1
        {
               k=((float)Right_Down_Point[1] - (float)Right_Up_Point[1])/((float)Right_Down_Point[0] - (float)Right_Up_Point[0]);
                     b=(float)Right_Up_Point[1] - ((float)Right_Up_Point[0]*k);
                for(uint8 y=Right_Up_Point[1];y<=Right_Down_Point[1];y++)
                              {
                               Right_Line[y]=(uint8)(((float)y-b)/k);  //两点之间补线
                               Right_Line[y]=limit_a_b(Right_Line[y],0,79);
                              }

           }

        if((float)Left_Down_Point[0] - (float)Left_Up_Point[0]!=0)
        {
                k=((float)Left_Down_Point[1] - (float)Left_Up_Point[1])/((float)Left_Down_Point[0] - (float)Left_Up_Point[0]);
                      b=(float)Left_Up_Point[1] - ((float)Left_Up_Point[0]*k);
                 for(uint8 y=Left_Up_Point[1];y<=Left_Down_Point[1];y++)
                               {
                     Left_Line[y]=(uint8)(((float)y-b)/k);  //两点之间补线
                     Left_Line[y]=limit_a_b(Left_Line[y],0,79);
                               }

        }
       /*  for(int i=0;i<=59;i++)
              {
                  Mid_Line[i] = (Left_Line[i] + Right_Line[i]) >> 1;
              } */
    }


    /***********************************同时找到上下拐点*****************************************************/


    /***********************************同时找到上拐点*****************************************************/
    else if((Right_Down_Point_finish_flag==0&&Right_Up_Point_finish_flag==1&&Left_Down_Point_finish_flag==0&&Left_Up_Point_finish_flag==1)
            ||(Right_Down_Point_finish_flag==1&&Right_Up_Point_finish_flag==1&&Left_Down_Point_finish_flag==0&&Left_Up_Point_finish_flag==1)
            ||(Right_Down_Point_finish_flag==0&&Right_Up_Point_finish_flag==1&&Left_Down_Point_finish_flag==1&&Left_Up_Point_finish_flag==1))
       {                                                                                     //1  1     1  1    1  1 
            nihexielv_up_r=Slope_Calculate(Right_Up_Point[1]-6,Right_Up_Point[1],Right_Line);//0  0     1  0    0  1
          if( nihexielv_up_r!=0)
          {     //printf("%d\r\n",nihexielv_up_r);
           k=(float)((float)1/nihexielv_up_r);
                     //b=(float)Right_Up_Point[1]-8 - ((float)Right_Line[Right_Up_Point[1]-8]*k);
           b=(float)Right_Up_Point[1]-6 - ((float)Right_Line[Right_Up_Point[1]-6]*k);
                     for(uint8 y=Right_Up_Point[1];y<=59;y++)
                        {

                         Right_Line[y]=(uint8)(((float)y-b)/k);  //两点之间补线
                         Right_Line[y]=limit_a_b(Right_Line[y],0,79);
                        }


       }    
            nihexielv_up_l=Slope_Calculate(Left_Up_Point[1]-6,Left_Up_Point[1],Left_Line);
          if(nihexielv_up_l!=0)
                  {
                 k=(float)((float)1/nihexielv_up_l);
                           //b=(float)Right_Up_Point[1]-8 - ((float)Right_Line[Right_Up_Point[1]-8]*k);
                 b=(float)Left_Up_Point[1]-6 - ((float)Left_Line[Left_Up_Point[1]-6]*k);
                           for(uint8 y=Left_Up_Point[1];y<=59;y++)
                              {

                               Left_Line[y]=(uint8)(((float)y-b)/k);  //两点之间补线
                               Left_Line[y]=limit_a_b(Left_Line[y],0,79);
                              }
                  }
          /* for(int i=0;i<=59;i++)
                {
                    Mid_Line[i] = (Left_Line[i] + Right_Line[i]) >> 1;
                } */

       }
    /***********************************同时找到上拐点*****************************************************/


    /***********************************只找到左下拐点*****************************************************/
    // else  if(Left_Down_Point_finish_flag==1&&Left_Up_Point_finish_flag==0)//0  0
    //   {                                                                   //1  0
    //     if(Slope_Calculate(Left_Down_Point[1],89,Left_Line)!=0)
    //    {
    //                  k=(float)((float)1/Slope_Calculate(Left_Down_Point[1],59,Left_Line));
    //                      b=(float)Left_Down_Point[1] - ((float)Left_Down_Point[0]*k);
    //                      for(uint8 y=0;y<=Left_Down_Point[1];y++)
    //                         {
    //                          Left_Line[y]=(uint8)(((float)y-b)/k);  //两点之间补线
    //                          Left_Line[y]=limit_a_b(Left_Line[y],0,79);
    //                          Right_Line[y] =Left_Line[y]+Standard_Road_Wide[y];
    //                          //Mid_Line[y]=limit_a_b(Mid_Line[y],0,159);

    //                         }

    //   }

    //   }

    /***********************************只找到左下拐点*****************************************************/


    /***********************************只找到左上下拐点*****************************************************/
    else  if(Left_Down_Point_finish_flag==1&&Left_Up_Point_finish_flag==1&&Left_Down_Point[1]>Left_Up_Point[1]+3)//1 0
              {                                                                                                  //1 0
            if((float)Left_Down_Point[0] - (float)Left_Up_Point[0]!=0)
                   {
                           k=((float)Left_Down_Point[1] - (float)Left_Up_Point[1])/((float)Left_Down_Point[0] - (float)Left_Up_Point[0]);
                           b=(float)Left_Up_Point[1] - ((float)Left_Up_Point[0]*k);
                            for(uint8 y=Left_Up_Point[1];y<=Left_Down_Point[1];y++)
                            {
                                Left_Line[y]=(uint8)(((float)y-b)/k);  //两点之间补线
                                Left_Line[y]=limit_a_b(Left_Line[y],0,79);
                                Right_Line[y] =Left_Line[y]+Standard_Road_Wide[y];
                               // Mid_Line[y]=limit_a_b(Mid_Line[y],0,159);
                            }

                   }

              }

        /***********************************只找到左上下拐点*****************************************************/

    //     /***********************************只找到左上拐点*****************************************************/
    // else     if(Left_Down_Point_finish_flag==0&&Left_Up_Point_finish_flag==1)
    //     {                                                                       //1 0
    //     if(Slope_Calculate(Left_Up_Point[1]-6,Left_Up_Point[1],Left_Line)!=0)   //0 0
    //                       {
    //                      k=(float)((float)1/Slope_Calculate(Left_Up_Point[1]-6,Left_Up_Point[1],Left_Line));
    //                                //b=(float)Right_Up_Point[1]-8 - ((float)Right_Line[Right_Up_Point[1]-8]*k);
    //                      b=(float)Left_Up_Point[1]-6 - ((float)Left_Line[Left_Up_Point[1]-6]*k);
    //                                for(uint8 y=Left_Up_Point[1];y<=59;y++)
    //                                   {

    //                                    Left_Line[y]=(int16)(((float)y-b)/k);  //两点之间补线
    //                                    Left_Line[y]=limit_a_b(Left_Line[y],0,79);
    //                                    Right_Line[y] =Left_Line[y]+Standard_Road_Wide[y];
    //                                    //Mid_Line[y]=limit_a_b(Mid_Line[y],0,159);
                                        
    //                                   }
    //                       }
    //     }
        /***********************************只找到左上拐点*****************************************************/

        /***********************************只找到右下拐点*****************************************************/
    // else   if(Right_Down_Point_finish_flag==1&&Right_Up_Point_finish_flag==0)//0 0
    //     {                                                                    //0 1
    //         if(Slope_Calculate(Right_Down_Point[1],89,Right_Line)!=0)
    //                {
    //                     k=(float)((float)1/Slope_Calculate(Right_Down_Point[1],59,Right_Line));

    //                         b=(float)Right_Down_Point[1] - ((float)Right_Down_Point[0]*k);
    //                         for(uint8 y=0;y<=Right_Down_Point[1];y++)
    //                            {
    //                             Right_Line[y]=(uint8)(((float)y-b)/k);  //两点之间补线
    //                             Right_Line[y]=limit_a_b(Right_Line[y],0,89);
    //                             Left_Line[y] =Right_Line[y]-Standard_Road_Wide[y];
    //                             //Mid_Line[y]=limit_a_b(Mid_Line[y],0,159);
    //                            }
    //                }

    //     }
        /***********************************只找到右下拐点*****************************************************/

        /***********************************只找到右上下拐点*****************************************************/
    else  if(Right_Down_Point_finish_flag==1&&Right_Up_Point_finish_flag==1&&Right_Down_Point[1]>Right_Up_Point[1]+3)
        {                                                                   //0 1
        if((float)Right_Down_Point[0] - (float)Right_Up_Point[0]!=0)        //0 1
             {
                    k=((float)Right_Down_Point[1] - (float)Right_Up_Point[1])/((float)Right_Down_Point[0] - (float)Right_Up_Point[0]);
                          b=(float)Right_Up_Point[1] - ((float)Right_Up_Point[0]*k);
                     for(uint8 y=Right_Up_Point[1];y<=Right_Down_Point[1];y++)
                                   {
                                    Right_Line[y]=(uint8)(((float)y-b)/k);  //两点之间补线
                                    Right_Line[y]=limit_a_b(Right_Line[y],0,89);
                                    Left_Line[y] =Right_Line[y]-Standard_Road_Wide[y];
                                    //Mid_Line[y]=limit_a_b(Mid_Line[y],0,159);
                                   }

                }
        }

        /***********************************只找到右上下拐点*****************************************************/


        /***********************************只找到右上拐点*****************************************************/
    // else if(Right_Down_Point_finish_flag==0&&Right_Up_Point_finish_flag==1)//0 1
    //     {                                                                  //0 0
    //         if( Slope_Calculate(Right_Up_Point[1]-6,Right_Up_Point[1],Right_Line)!=0)
    //                {
    //                 k=(float)((float)1/Slope_Calculate(Right_Up_Point[1]-6,Right_Up_Point[1],Right_Line));
    //                           //b=(float)Right_Up_Point[1]-8 - ((float)Right_Line[Right_Up_Point[1]-8]*k);
    //                 b=(float)Right_Up_Point[1]-6 - ((float)Right_Line[Right_Up_Point[1]-6]*k);
    //                           for(uint8 y=Right_Up_Point[1];y<=59;y++)
    //                              {

    //                               Right_Line[y]=(uint8)(((float)y-b)/k);  //两点之间补线
    //                               Right_Line[y]=limit_a_b(Right_Line[y],0,79);
    //                               Left_Line[y] =Right_Line[y]-Standard_Road_Wide[y];
    //                              // Mid_Line[y]=limit_a_b(Mid_Line[y],0,159);
    //                              }


    //             }

    //     }

        /***********************************只找到右上拐点*****************************************************/

    }
}
}



/********十字********/




void Get_Mid()
{   int i;
    int y;int z;int x;
    float k1=0;float k=0;float b=0;

for(i=UVC_H-Search_Stop_Line;i<=UVC_H-1;i++)
        {
            Mid_Line[i]=(Right_Line[i]+Left_Line[i])/2;
            
            if(Mid_Line[i]>79)Mid_Line[i]=79;
            if(Mid_Line[i]<0)Mid_Line[i]=0;
            
            
 
        }
//          if(Island_State==2||Island_State==3)
//     {
//         for(i=UVC_H-Search_Stop_Line;i>0;i--)
//         {   
//             Mid_Line[i]=(Mid_Line[i+1]+Mid_Line[i+3])/2;
//             if(Mid_Line[i]<0)Mid_Line[i]=0;
//             if(Mid_Line[i]>160)Mid_Line[i]=160;
//         //ips200_draw_point(Mid_Line[i], i,  0xF800); 
        

//         }
//     }
//    else
//     {   
    if(UVC_H-Search_Stop_Line<=44)
        {k1=Slope_Calculate(UVC_H-Search_Stop_Line+5,UVC_H-Search_Stop_Line+15,Mid_Line);
        if(k1!=0)
            {
             k=((float)1.0/k1);

                 b=(float)(UVC_H-Search_Stop_Line+10)- ((float)Mid_Line[UVC_H-Search_Stop_Line+10]*k);

                 for( y=UVC_H-Search_Stop_Line+10;y>=0;y--)
              {
               Mid_Line[y]=(uint8)(((float)y-b)/k);  //两点之间补线 


               if(Mid_Line[y]<=2)
                                                            {//两点之间补线
                                                                for( z=y;z>0;z--)
                                                                {
                                                                    Mid_Line[z]=0;
                                                                }
                                                                break;
                                                            }

               else                           if(Mid_Line[y]>=79)
                                                            {//两点之间补线
                                                                for( x=y;x>0;x--)
                                                                {
                                                                    Mid_Line[x]=79;
                                                                }
                                                                break;
                                                            }
                                                            



              }

            }

    }
}


/*-------------------------------------------------------------------------------------------------------------------
  @brief     单调性突变检测
  @param     起始点，终止行
  @return    点所在的行数，找不到返回0
  Sample     Find_Right_Up_Point(int start,int end);
  @note      前5后5它最大（最小），那他就是角点
-------------------------------------------------------------------------------------------------------------------*/
int Monotonicity_Change_Right(int start,int end)//单调性改变，返回值是单调性改变点所在的行数
{
    int i;
    int monotonicity_change_line=0;
 
    if(Right_Lost_Time>=0.9*UVC_H)//大部分都丢线，没有单调性判断的意义
        return monotonicity_change_line;
    if(start>=UVC_H-1-5)//数组越界保护
        start=UVC_H-1-5;
     if(end<=5)
        end=5;
    if(start<=end)
        return monotonicity_change_line;
    for(i=start;i>=end;i--)//会读取前5后5数据，所以前面对输入范围有要求
    {
        if(Right_Line[i]==Right_Line[i+5]&&Right_Line[i]==Right_Line[i-5]&&
        Right_Line[i]==Right_Line[i+4]&&Right_Line[i]==Right_Line[i-4]&&
        Right_Line[i]==Right_Line[i+3]&&Right_Line[i]==Right_Line[i-3]&&
        Right_Line[i]==Right_Line[i+2]&&Right_Line[i]==Right_Line[i-2]&&
        Right_Line[i]==Right_Line[i+1]&&Right_Line[i]==Right_Line[i-1])
        {//一堆数据一样，显然不能作为单调转折点
            continue;
        }
        else if(Right_Line[i] <Right_Line[i+5]&&Right_Line[i] <Right_Line[i-5]&&
        Right_Line[i] <Right_Line[i+4]&&Right_Line[i] <Right_Line[i-4]&&
        Right_Line[i]<=Right_Line[i+3]&&Right_Line[i]<=Right_Line[i-3]&&
        Right_Line[i]<=Right_Line[i+2]&&Right_Line[i]<=Right_Line[i-2]&&
        Right_Line[i]<=Right_Line[i+1]&&Right_Line[i]<=Right_Line[i-1])
        {//就很暴力，这个数据是在前5，后5中最大的，那就是单调突变点
            monotonicity_change_line=i;
            break;
        }
    }
    return monotonicity_change_line;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     单调性突变检测
  @param     起始点，终止行
  @return    点所在的行数，找不到返回0
  Sample     Find_Right_Up_Point(int start,int end);
  @note      前5后5它最大（最小），那他就是角点
-------------------------------------------------------------------------------------------------------------------*/
int Monotonicity_Change_Left(int start,int end)//单调性改变，返回值是单调性改变点所在的行数
{
    int i;
    int monotonicity_change_line=0;
    if(Left_Lost_Time>=0.9*UVC_H)//大部分都丢线，没有单调性判断的意义
       return monotonicity_change_line;
    if(start>=UVC_H-1-5)//数组越界保护，在判断第i个点时
       start=UVC_H-1-5; //要访问它前后5个点，数组两头的点要不能作为起点终点
    if(end<=5)
        end=5;
    if(start<=end)//递减计算，入口反了，直接返回0
      return monotonicity_change_line;
    for(i=start;i>=end;i--)//会读取前5后5数据，所以前面对输入范围有要求
    {
        if(Left_Line[i]==Left_Line[i+5]&&Left_Line[i]==Left_Line[i-5]&&
        Left_Line[i]==Left_Line[i+4]&&Left_Line[i]==Left_Line[i-4]&&
        Left_Line[i]==Left_Line[i+3]&&Left_Line[i]==Left_Line[i-3]&&
        Left_Line[i]==Left_Line[i+2]&&Left_Line[i]==Left_Line[i-2]&&
        Left_Line[i]==Left_Line[i+1]&&Left_Line[i]==Left_Line[i-1])
        {//一堆数据一样，显然不能作为单调转折点
            continue;
        }
        else if(Left_Line[i]>=Left_Line[i+5]&&Left_Line[i]>=Left_Line[i-5]&&
        Left_Line[i]>=Left_Line[i+4]&&Left_Line[i]>=Left_Line[i-4]&&
        Left_Line[i]>=Left_Line[i+3]&&Left_Line[i]>=Left_Line[i-3]&&
        Left_Line[i]>=Left_Line[i+2]&&Left_Line[i]>=Left_Line[i-2]&&
        Left_Line[i]>=Left_Line[i+1]&&Left_Line[i]>=Left_Line[i-1])
        {//就很暴力，这个数据是在前5，后5中最大的（可以取等），那就是单调突变点
            monotonicity_change_line=i;
            break;
        }
    }
    return monotonicity_change_line;
}

int GetDownLeft(int y1,int y2)  //获取左下拐点信息
{
    uint8 findl_x = 0;    //左点
    uint8 findl_y = 0;
    uint8 examl_x = 0;
    uint8 examl_y = 0;
    uint8 examcount = 0;
		int8 directionlu[5][2] =  {{1,0}, {1,-1}, {0,-1}, {-1,-1}, {1,1}};
		uint8 y;
		uint8 dircount;
    if(DownLeftFlag==1)
            DownLeftFlag=0;
    
    for( y=y1;y>=y2;y--)
        {

            if(DownLeftFlag==0 )
            {
                if(Left_Lost_Flag[y]==1)
                {
                //gpio_set_level(BEEP_PIN,1);
                    findl_x=Left_Line[y];
                    findl_y=y;

                for( dircount = 0;dircount<5;dircount++)
                                       {
                    examl_x=findl_x+directionlu[dircount][0];
                    examl_y=findl_y+directionlu[dircount][1];
                                           if(image_two_value[examl_y][examl_x]==255)
                                           {
                                               examcount++;
                                           }
                                       }

                                       if(examcount>=4)
                                       {//&&(Left_Line[findl_y]-Left_Line[findl_y-1]>=5||Left_Line[findl_y]-Left_Line[findl_y-2]>=5)

										 if((Left_Line[findl_y]-Left_Line[findl_y-1]>=5&&Left_Line[findl_y]-Left_Line[findl_y-2]>=5))
												 {examcount=0;																		
                                           DownLeft[0]=findl_y;
                                           DownLeft[1]=findl_x;
                                           DownLeftFlag=1;
                                         break;}
																				 else 
																				 {
																					 DownLeftFlag = 0;
                                            examcount=0;
																				 
																				 }
																					 

                                       }

                                       else
                                        {
                                           DownLeftFlag = 0;
                                            examcount=0;
                                            
                                        }



            }

        }
        }

return DownLeftFlag;

}

int GetDownRight(int y1,int y2)
{
    uint8 findr_x = 0;    //右点
    uint8 findr_y = 0;
    uint8 examr_x = 0;
    uint8 examr_y = 0;
    uint8 examcount = 0;
    int8 directionld[5][2] =  {{-1,0}, {-1,-1}, {0,-1}, {1,-1}, {1,-1}};
		uint8 y;
		uint8 dircount;
    if(DownRightFlag==1)
            DownRightFlag=0;
    for(y=y1;y>=y2;y--)
       {

           if(DownRightFlag==0 )
           {
               if(Right_Lost_Flag[y]==1)
               {
               //gpio_set_level(BEEP_PIN,1);
               findr_x=Right_Line[y];
               findr_y=y;

               for( dircount = 0;dircount<5;dircount++)
                                      {
                                          examr_x=findr_x+directionld[dircount][0];
                                          examr_y=findr_y+directionld[dircount][1];
                                          if(image_two_value[examr_y][examr_x]==255)
                                          {
                                              examcount++;
                                          }
                                      }
                                      if(examcount>=4)
                                      {//
									if((Right_Line[findr_y-1]-Right_Line[findr_y]>=5&&Right_Line[findr_y-2]-Right_Line[findr_y]>=5))
											{examcount=0;																				
                                          DownRight[0]=findr_y;
                                          DownRight[1]=findr_x;
                                          DownRightFlag=1;
                                        break;}
										else 
										{
											DownRightFlag = 0;																													
                                           examcount=0;
																				}
                                      }
                                      else
                                       {
                                          DownRightFlag = 0;
                                           examcount=0;
                                           
                                       }
               }

           }

       }
    return DownRightFlag;
}

int GetUpLeft(int y1,int y2)//10,59
{
    uint8 findl_x = 0;    //左点
        uint8 findl_y = 0;
        uint8 examl_x = 0;
        uint8 examl_y = 0;
        uint8 examcount = 0;
				int8 directionlu[5][2] =  {{1,0}, {1,1}, {0,1}, {-1,1}, {-1,1}};
        uint8 y;
				uint8 dircount;
				if(UpLeftFlag==1)
            UpLeftFlag=0;
        
        for( y=y1;y<=y2;y++)
            {

                if(UpLeftFlag==0 )
                {
                    if(Left_Lost_Flag[y]==1)
                    {
                    //gpio_set_level(BEEP_PIN,1);
                        findl_x=Left_Line[y];
                        findl_y=y;

                    for( dircount = 0;dircount<5;dircount++)
                                           {
                        examl_x=findl_x+directionlu[dircount][0];
                        examl_y=findl_y+directionlu[dircount][1];
                                               if(image_two_value[examl_y][examl_x]==255)
                                               {
                                                   examcount++;
                                               }
                                           }

                                           if(examcount>=4)
                                            {
												if((Left_Line[findl_y]-Left_Line[findl_y+1]>=5||Left_Line[findl_y]-Left_Line[findl_y+2]>=5))
												 //&&Left_Line[findl_y-1]-Left_Line[findl_y]<=4&&Left_Line[findl_y-2]-Left_Line[findl_y-1]<=4)																				
                                           {  examcount=0;
                                               UpLeft[0]=findl_y;
                                               UpLeft[1]=findl_x;
                                               UpLeftFlag=1;
                                             break;}
																					 else 
																					 {
																						 UpLeftFlag = 0;
                                                examcount=0;
																					 }

                                           }

                                           else
                                            {
                                               UpLeftFlag = 0;
                                                examcount=0;
                                               
                                            }



                }

            }

}
return UpLeftFlag;
}

int GetUpRight(int y1,int y2)
{
    uint8 findr_x = 0;    //右点
    uint8 findr_y = 0;
    uint8 examr_x = 0;
    uint8 examr_y = 0;
    uint8 examcount = 0;
    int8 directionld[5][2] =  {{-1,0}, {-1,1}, {0,1}, {1,1}, {1,1}};
    uint8 y;
		uint8 dircount;
		if(UpRightFlag==1)
            UpRightFlag=0;
    for( y=y1;y<=y2;y++)
       {

           if(UpRightFlag==0 )
           {
               if(Right_Lost_Flag[y]==1)
               {
               //gpio_set_level(BEEP_PIN,1);
               findr_x=Right_Line[y];
               findr_y=y;

               for( dircount = 0;dircount<5;dircount++)
                                      {
                                          examr_x=findr_x+directionld[dircount][0];
                                          examr_y=findr_y+directionld[dircount][1];
                                          if(image_two_value[examr_y][examr_x]==255)
                                          {
                                              examcount++;
                                          }
                                      }
                                      if(examcount>=4)
																			{	
                                        if((Right_Line[findr_y+1]-Right_Line[findr_y]>=5||Right_Line[findr_y+2]-Right_Line[findr_y]>=5))
                                    //&&Right_Line[findr_y]-Right_Line[findr_y-1]<=4&&Right_Line[findr_y-1]-Right_Line[findr_y-2]<=4)
                                      {
                                          examcount=0;
                                          UpRight[0]=findr_y;
                                          UpRight[1]=findr_x;
                                          UpRightFlag=1;
                                        break;
                                      }
																			else
																			{
																				UpRightFlag = 0;
                                           examcount=0;
																				
																			}
																			}
                                      else
                                       {
                                          UpRightFlag = 0;
                                           examcount=0;
                                          
                                       }
               }

           }

       }
    return UpRightFlag;
}


void huandaoyoudown(void)
{

if(youxia==1)
    youxia=0;

if(youxia==0&&DownRightFlag==1)
{
    if(abs(Right_Line[DownRight[0]]-Right_Line[DownRight[0]-1])>8)
    {
    youxia=1;
    }

}
}

void huandaozuodown(void)
{
if(zuoxia==1)
    zuoxia=0;

if(zuoxia==0&&DownLeftFlag==1)
{
    if(abs(Left_Line[DownLeft[0]]-Left_Line[DownLeft[0]-1])>8)
    {
    zuoxia=1;
    }

}

}


void huandaozuoup(void)
{
if(zuoup==1)
    zuoup=0;
if(zuoup==0&&UpLeftFlag==1)
{
    if(abs(Left_Line[UpLeft[0]]-Left_Line[UpLeft[0]+1])||abs(Left_Line[UpLeft[0]]-Left_Line[UpLeft[0]+2])>8)
    {
    zuoup=1;
    }

}

}
void huandaoyouup(void)
{
if(youup==1)
    youup=0;
if(youup==0&&UpRightFlag==1)
{
    if(abs(Right_Line[UpRight[0]]-Right_Line[UpRight[0]+1])>8)
    {
    youup=1;
    }

}

}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     通过斜率，定点补线--
  @param     k       输入斜率
             startY  输入起始点纵坐标
             endY    结束点纵坐标
  @return    null
  Sample     K_Add_Boundry_Left(float k,int startY,int endY);
  @note      补得线直接贴在边线上
-------------------------------------------------------------------------------------------------------------------*/
void K_Add_Boundry_Left(float k,int startX,int startY,int endY)
{
    int i,t;
    if(startY>=UVC_H-1)
        startY=UVC_H-1;
    else if(startY<=0)
        startY=0;
    if(endY>=UVC_H-1)
        endY=UVC_H-1;
    else if(endY<=0)
        endY=0;
    if(startY<endY)//--操作，start需要大
    {
        t=startY;
        startY=endY;
        endY=t;
    }
//这里有bug，下方循环--循环，需要start要大，只进行y的互换，但是没有进行x的互换
//建议进行判断，如果start更小，那就进行++访问
//这里修改各位自行操作
    for(i=startY;i>=endY;i--)
    {
        Left_Line[i]=(int)((i-startY)/k+startX);//(y-y1)=k(x-x1)变形，x=(y-y1)/k+x1
        if(Left_Line[i]>=UVC_W-1)
        {
            Left_Line[i]=UVC_W-1;
        }
        else if(Left_Line[i]<=0)
        {
            Left_Line[i]=0;
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     通过斜率，定点补线
  @param     k       输入斜率
             startY  输入起始点纵坐标
             endY    结束点纵坐标
  @return    null    直接补边线
  Sample     K_Add_Boundry_Right(float k,int startY,int endY);
  @note      补得线直接贴在边线上
-------------------------------------------------------------------------------------------------------------------*/
void K_Add_Boundry_Right(float k,int startX,int startY,int endY)
{
    int i,t;
    if(startY>=UVC_H-1)
        startY=UVC_H-1;
    else if(startY<=0)
        startY=0;
    if(endY>=UVC_H-1)
        endY=UVC_H-1;
    else if(endY<=0)
        endY=0;
    if(startY<endY)
    {
        t=startY;
        startY=endY;
        endY=t;
    }
    for(i=startY;i>=endY;i--)
    {
        Right_Line[i]=(int)((i-startY)/k+startX);//(y-y1)=k(x-x1)变形，x=(y-y1)/k+x1
        if(Right_Line[i]>=UVC_W-1)
        {
            Right_Line[i]=UVC_W-1;
        }
        else if(Right_Line[i]<=0)
        {
            Right_Line[i]=0;
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     画线
  @param     输入起始点，终点坐标，补一条宽度为2的黑线
  @return    null
  Sample     Draw_Line(0, 0,MT9V03X_W-1,MT9V03X_H-1);
             Draw_Line(MT9V03X_W-1, 0,0,MT9V03X_H-1);
                                    画一个大×
  @note     补的就是一条线，需要重新扫线
-------------------------------------------------------------------------------------------------------------------*/
void Draw_Line(int startX, int startY, int endX, int endY)
{
    int i,x,y;
    int start=0,end=0;
    if(startX>=UVC_W-1)//限幅处理
        startX=UVC_W-1;
    else if(startX<=0)
        startX=0;
    if(startY>=UVC_H-1)
        startY=UVC_H-1;
    else if(startY<=0)
        startY=0;
    if(endX>=UVC_W-1)
        endX=UVC_W-1;
    else if(endX<=0)
        endX=0;
    if(endY>=UVC_H-1)
        endY=UVC_H-1;
    else if(endY<=0)
        endY=0;
    if(startX==endX)//一条竖线
    {
        if (startY > endY)//互换
        {
            start=endY;
            end=startY;
        }
        for (i = start; i <= end; i++)
        {
            if(i<=1)
                i=1;
            image_two_value[i][startX]=IMG_BLACK;
            image_two_value[i-1][startX]=IMG_BLACK;
        }
    }
    else if(startY == endY)//补一条横线
    {
        if (startX > endX)//互换
        {
            start=endX;
            end=startX;
        }
        for (i = start; i <= end; i++)
        {
            if(startY<=1)
                startY=1;
            image_two_value[startY][i]=IMG_BLACK;
            image_two_value[startY-1][i]=IMG_BLACK;
        }
    }
    else //上面两个是水平，竖直特殊情况，下面是常见情况
    {
        if(startY>endY)//起始点矫正
        {
            start=endY;
            end=startY;
        }
        else
        {
            start=startY;
            end=endY;
        }
        for (i = start; i <= end; i++)//纵向补线，保证每一行都有黑点
        {
            x =(int)(startX+(endX-startX)*(i-startY)/(endY-startY));//两点式变形
            if(x>=UVC_W-1)
                x=UVC_W-1;
            else if (x<=1)
                x=1;
            image_two_value[i][x] = IMG_BLACK;
            image_two_value[i][x-1] = IMG_BLACK;
        }
        if(startX>endX)
        {
            start=endX;
            end=startX;
        }
        else
        {
            start=startX;
            end=endX;
        }
        for (i = start; i <= end; i++)//横向补线，保证每一列都有黑点
        {
 
            y =(int)(startY+(endY-startY)*(i-startX)/(endX-startX));//两点式变形
            if(y>=UVC_H-1)
                y=UVC_H-1;
            else if (y<=0)
                y=0;
            image_two_value[y][i] = IMG_BLACK;
        }
    }
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     根据斜率划线
  @param     输入斜率，定点，画一条黑线
  @return    null
  Sample     K_Draw_Line(k, 20,MT9V03X_H-1 ,0)
  @note      补的就是一条线，需要重新扫线
-------------------------------------------------------------------------------------------------------------------*/
void K_Draw_Line(float k, int startX, int startY,int endY)
{
    int endX=0;
 
    if(startX>=UVC_W-1)//限幅处理
        startX=UVC_W-1;
    else if(startX<=0)
        startX=0;
    if(startY>=UVC_H-1)
        startY=UVC_H-1;
    else if(startY<=0)
        startY=0;
    if(endY>=UVC_H-1)
        endY=UVC_H-1;
    else if(endY<=0)
        endY=0;
    endX=(int)((endY-startY)/k+startX);//(y-y1)=k(x-x1)变形，x=(y-y1)/k+x1
    Draw_Line(startX,startY,endX,endY);
}
int Get_High(int x)
{
    int i,y;
for(i=55;i>=10;i--){
if (image_two_value[i][x]==255&&image_two_value[i-1][x]==255&&image_two_value[i-2][x]==0&&image_two_value[i-3][x]==0&&i-2<=45)
   {
    y=i-2;
    break;
    }
else

{ y=0;

}
}

return y;
}


void Island_Detect()
{   int i=0;
    static float k_L=0;//3和5状态的k
    static float k_R=0;
    static int8 island_state_5_down[2]={0};//状态5时即将离开环岛，左右边界边最低点，[0]存y，第某行，{1}存x，第某列
    static int8 island_state_3_up[2]={0};//状态3时即将进入环岛用，左右上面角点[0]存y，第某行，{1}存x，第某列
    static int8 left_down_guai[2]={0};//四个拐点的坐标存储，[0]存y，第某行，{1}存x，第某列
    static int8 right_down_guai[2]={0};//四个拐点的坐标存储，[0]存y，第某行，{1}存x，第某列
//    int monotonicity_change_line[2]={0};//单调性改变点坐标，[0]寸某行，[1]寸某列
    int8 monotonicity_change_left_flag=0;//不转折是0
    int8 monotonicity_change_right_flag=0;//不转折是0
    int8 continuity_change_right_flag=0; //连续是0
    int8 continuity_change_left_flag=0;  //连续是0
    //以下是常规判断法
    int8 t=0;
    
   /*  continuity_change_left_flag=Continuity_Change_Left(UVC_H-1-5,10);//连续性判断
    continuity_change_right_flag=Continuity_Change_Right(UVC_H-1-5,10); */
    /* if(Island_State!=0)
    {
        Island_0=0;
    } */

   //printf("Island_State=%d.\r\n",Island_State);
    if(Island_State==0&&LeftzhuantouFlag==0&&RightzhuantouFlag==0&&Cross_Flag==0)//&&Ramp_Flag==0Cross_Flag==0&&
    {
        continuity_change_left_flag=Continuity_Change_Left(UVC_H-1-8,10);//连续性判断
        continuity_change_right_flag=Continuity_Change_Right(UVC_H-1-8,10);
        monotonicity_change_right_flag=Monotonicity_Change_Right(UVC_H-1-8,10);//单调性判断
        monotonicity_change_left_flag=Monotonicity_Change_Left(UVC_H-1-8,10);
        // printf("%d\r\n",continuity_change_right_flag);
        if(Left_Island_Flag==0)//左环
        {   //printf("%d\r\n",Left_Lost_Time);
            //left_down_guai[0]=Find_Left_Down_Point(UVC_H-1,20);//找左下角点
            DownLeftFlag=GetDownLeft(UVC_H-1-5,15);
            //huandaozuodown();
            // printf("DownLeft[0]=%d.\r\n",DownLeft[0]);
            // printf("DownLeft[1]=%d.\r\n",DownLeft[1]);
                
            if(monotonicity_change_right_flag==0&& //右边是单调的
               continuity_change_left_flag!=0&& //左边是不连续的
               continuity_change_right_flag==0&& //左环岛右边是连续的
               Left_Lost_Time>=7&& //左边丢线很多
               Left_Lost_Time<=50&& //也不能全丢了
               Right_Lost_Time<=7&&//右边丢线较少
               Search_Stop_Line>=UVC_H*0.8&& //搜索截止行看到很远
               Boundry_Start_Left>=UVC_H-20&&Boundry_Start_Right>=UVC_H-20&& //边界起始点靠下
               Both_Lost_Time<=7&&Right_Down_Point_finish_flag==0&&abs(continuity_change_left_flag-DownLeft[0])<=6)//双边丢线少
            {   //printf("%d\r\n",left_down_guai[0]);&&zuoxia==1
               
                if(DownLeft[0]>=20)//条件1很松，在这里判断拐点，位置不对，则是误判，跳出
                {
                     Island_State=1;//这里有问题
                    Left_Island_Flag=1;
                }
                else//误判，归零
                {
                    Island_State=0;
                    Left_Island_Flag=0;
                }
            }
            if(monotonicity_change_right_flag==0&& //右边是单调的
               continuity_change_left_flag!=0&& //左边是不连续的
               continuity_change_right_flag==0&&//左环岛右边是连续的
                Search_Stop_Line>=UVC_H*0.90&&Left_Lost_Time>=5)
            {
                for(i=0;i<=UVC_H-1;i++)
            {
                Mid_Line[i]=Right_Line[i]-Standard_Road_Wide[i]/2;
                
            }
            

            }
            
        }

                if(Right_Island_Flag==0)//右环
        {       
/***************改到这***************/
            //right_down_guai[0]=Find_Right_Down_Point(UVC_H-1,20);//找右下角点
            DownRightFlag=GetDownRight(UVC_H-1-5,10);
            //huandaoyoudown();
           
//printf("%d\r\n",Both_Lost_Time);
            if(monotonicity_change_left_flag==0&&
               continuity_change_left_flag==0&& //右环岛左边是连续的
               continuity_change_right_flag!=0&& //右边是不连续的
               Right_Lost_Time>=7&&           //右丢线多
               Right_Lost_Time<=50&&           //右丢线不能太多
               Left_Lost_Time<=7&&            //左丢线少
               Search_Stop_Line>=UVC_H*0.8&& //搜索截止行看到很远
               Boundry_Start_Left>=UVC_H-20&&Boundry_Start_Right>=UVC_H-20&& //边界起始点靠下
               Both_Lost_Time<=7&&Left_Down_Point_finish_flag==0&&abs(continuity_change_right_flag-DownRight[0])<=6)
            {   
                    
                //right_down_guai[0]=Find_Right_Down_Point(UVC_H-1,20);//右下点
                if(DownRight[0]>=20)//条件1很松，在这里加判拐点，位置不对，则是误判，跳出&&youxia==1
                {
                    Island_State=1;
                    Right_Island_Flag=1;

                }
                else
                {
                    Island_State=0;
                    Right_Island_Flag=0;
                }
            } 
            if(monotonicity_change_left_flag==0&& //左边是单调的
               continuity_change_right_flag!=0&& //右边是不连续的
               continuity_change_left_flag==0&&//右环岛左边是连续的
               Search_Stop_Line>=UVC_H*0.90&&Right_Lost_Time>=5)
            {
                for(i=0;i<=UVC_H-1;i++)
            {
               Mid_Line[i] =Left_Line[i]+Standard_Road_Wide[i]/2;
                
            }
                
            }
           
        }

    }
       


    if(Left_Island_Flag==1)//1状态下拐点还在，没丢线
    {
    {
        if(Island_State==1)
        {  //Zebra_Stripes_Flag==2;
            /* monotonicity_change_line[0]=Monotonicity_Change_Left(30,5);//寻找单调性改变点
            monotonicity_change_line[1]=Left_Line[monotonicity_change_line[0]]; */
            //printf("%d\r\n",monotonicity_change_line[0]);
            //Left_Add_Line((int)(monotonicity_change_line[1]*0.15),UVC_H-1,monotonicity_change_line[1],monotonicity_change_line[0]);
                  for(i=0;i<=UVC_H-1;i++)
             {
                 Mid_Line[i]=Right_Line[i]-Standard_Road_Wide[i]/2;
                
            } 
           /* for(i=0;i<=UVC_H-1;i++)
            {
                Left_Line[i]=Right_Line[i]-Standard_Road_Wide[i];
                
            } */ 
            
            if((Island_State==1)&&(Boundry_Start_Left<30))//下方当丢线时候进2
            {//这个38需要重新测
                Island_State=2;
            }
        }
    
        if(Island_State==2)//下方角点消失，2状态时下方应该是丢线，上面是弧线
        {
            /* monotonicity_change_line[0]=Monotonicity_Change_Left(60,10);//寻找单调性改变点
            monotonicity_change_line[1]=Left_Line[monotonicity_change_line[0]]; */
            //printf("%d\r\n",Boundry_Start_Left);
            //printf("%d\r\n",monotonicity_change_line[0]);
            //Left_Add_Line((int)(monotonicity_change_line[1]*0.1),UVC_H-1,monotonicity_change_line[1],monotonicity_change_line[0]);
             for(i=0;i<=UVC_H-1;i++)
            {
                Mid_Line[i]=Right_Line[i]-Standard_Road_Wide[i]/2;
                
            } 
            if(Island_State==2&&(Boundry_Start_Left>=UVC_H-5)&&motor.huandao_L2>25)//当圆弧靠下时候，进3||monotonicity_change_line[0]>45)&&motor.huandao_L2>25
            {
                Island_State=3;//最长白列寻找范围也要改，见camera.c
                Left_Island_Flag=1;
            }
            //编码器积分未加
        }
         if(Island_State==3)//3状态准备进环，寻找上拐点，连线
        {   

            UpLeftFlag=GetUpLeft(10,50);
            //huandaozuoup();
					if(UpLeftFlag==1)
					{     //y                                          //x
						if((10<= UpLeft[0]&& UpLeft[0]<50)&&(10< UpLeft[1]&& UpLeft[1]<70))//此处参数要重写
              { //printf("%d\r\n",1);
                //Draw_Line(93,59,UpLeft[1],UpLeft[0]);
               //Longest_White_Column(1);//最长白列
                //Padding_CurveR(UpLeft[1],UpLeft[0],140,60);
               Draw_Line(70,59,UpLeft[1],UpLeft[0]);
               Longest_White_Column(1);
                Get_Mid();
                 /* for(i=0;i<=25;i++)
                     {   if(Mid_Line[i]>=70)
                         Mid_Line[i]=70;

                     } */


               }
					}
						
					if(UpLeftFlag==0)
					{
					
					if((10<= UpLeft[0]&& UpLeft[0]<50)&&(10< UpLeft[1]&& UpLeft[1]<70))//此处参数要重写
              { //printf("%d\r\n",1);
                //Draw_Line(80,59,UpLeft[1]-5,UpLeft[0]-5);
               //Longest_White_Column(1);//最长白列
               //Padding_CurveR(UpLeft[1]+5,UpLeft[0]+5,140,60);
                Draw_Line(70,59,UpLeft[1]-5,UpLeft[0]-5);
               Longest_White_Column(1);//最长白列
                Get_Mid();
                 /* for(i=0;i<=25;i++)
                     {   if(Mid_Line[i]>=70)
                         Mid_Line[i]=70;

                     } */

               }
					
					}
           
            // if(k_L!=0)
            // {  
            //     K_Draw_Line(k_L,UVC_W-40,UVC_H-1,0);//k是刚刚算出来的，静态变量存着
            //     Longest_White_Column_Left_huan();//刷新边界数据
            //     Get_Mid();
            //     for(i=10;i<=25;i++)
            //         {   if(Mid_Line[i]>=75)
            //             Mid_Line[i]=75;

            //         }
            // }
            // else
            // {
                 /* Left_Up_Guai[0]=Find_Left_up_Point(5,45);//找左上拐点
                 Left_Up_Guai[1]=Left_Line[Left_Up_Guai[0]]; */
                 //UpLeftFlag=GetUpLeft(5,55);
                // huandaozuoup();
               // printf("%d\r\n",Left_Up_Guai[0]);
                /* if (Left_Up_Guai[0]<5)//此处为了防止误判，如果经常从3状态归零，建议修改此处判断条件
                {
                    Island_State=0;
                    Left_Island_Flag=0;
                } */

            //     if(k_L==0&&(7<= UpLeft[0]&& UpLeft[0]<50)&&(35< UpLeft[1]&& UpLeft[1]<120))//拐点出现在一定范围内，认为是拐点出现
            //     {
            //         island_state_3_up[0]=  UpLeft[0]+5;
            //         island_state_3_up[1]=  UpLeft[1]-5;
            //         k_L=(float)((float)(UVC_H-island_state_3_up[0])/(float)(UVC_W-40-island_state_3_up[1]));
            //         K_Draw_Line(k_L,UVC_W-40,UVC_H-1,0);//记录下第一次上点出现时位置，针对这个环岛拉一条死线，入环
            //         //Image_Binarization(threshold); 
            //         //printf("%f\r\n",k);
            //         //Longest_White_Column();//刷新边界数据
            //         Longest_White_Column_Left_huan();
            //         Get_Mid();
            //         for(i=10;i<=25;i++)
            //         {   if(Mid_Line[i]>=75)
            //             Mid_Line[i]=75;

            //         } 
                    
            //     }
            //     /* Longest_White_Column();
            //         Get_Mid();
            //         for(i=0;i<=20;i++)
            //         {   if(Mid_Line[i]>=80)
            //             Mid_Line[i]=80;

            //         }*/ 
            // }
            if((Island_State==3)&&(abs(icmdata.huandao_yaw)>=45)&&(abs(icmdata.huandao_yaw)<=100))//纯靠陀螺仪积分入环
            {
                k_L=0;//斜率清零
                Island_State=4;//这一步时候顺便调整了最长白列的搜索范围
                /* Longest_White_Column();//最长白列
                Get_Mid(); */
            } 
        }
    }

        if(Island_State==4)//状态4已经在里面
        {
            //积分200度以后在打开出环判断if(abs(icmdata.Yaw_youhuan1)>200)
//                for(i=10;i<=59;i++)
//                    {   if(Mid_Line[i]>=47)
//                        Mid_Line[i]=47;

//                    }
                // monotonicity_change_line[0]=Monotonicity_Change_Right(UVC_H-10,10);//单调性改变
                // monotonicity_change_line[1]=Right_Line[monotonicity_change_line[0]];//这里应该改成角点
               
               //if(abs(icmdata.huandao_yaw)>200)
             //{   //monotonicity_change_line[0]=Monotonicity_Change_Right(UVC_H-10,10);//单调性改变
            //     monotonicity_change_line[1]=Right_Line[monotonicity_change_line[0]];
                DownRightFlag=GetDownRight(55,10);

                if((Island_State==4)&&(abs(icmdata.huandao_yaw)>240)&&(DownRightFlag==1&&10<=DownRight[0]&&DownRight[0]<=50&&DownRight[1]>=20))//单调点靠下，进去5
                {	//此处参数也需要重写
									//monotonicity_change_line[1]>=90&&
                    /* island_state_5_down[0]=UVC_H-1;
                    island_state_5_down[1]=Right_Line[UVC_H-1]; */
                     /* k=(float)((float)(UVC_H-monotonicity_change_line[0])/(float)(island_state_5_down[1]-monotonicity_change_line[1]));
                     K_Add_Boundry_Right(k,island_state_5_down[1],island_state_5_down[0],0); *///和状态3一样，记住斜率
                    Island_State=5;
                   /*  Draw_Line(35,Get_High(35),DownRight[1]+2,DownRight[0]);
                    Longest_White_Column();
                    Get_Mid();
                    for(i=10;i<=20;i++)
                    {   if(Mid_Line[i]>=80)
                        Mid_Line[i]=80;

                    } */
                    
                }              
             //}
        }
        if(Island_State==5)//出环
        {   
            DownRightFlag=GetDownRight(58,10);
            //K_Add_Boundry_Right(k,island_state_5_down[1],island_state_5_down[0],0);
            //Draw_Line(10,Get_High(10),DownRight[1]+2,DownRight[0]);
            // if((10<=DownRight[0]&&DownRight[0]<=58&&DownRight[1]>=20))
            // {
                
            // }
            Draw_Line(20,Get_High(20),DownRight[1]+2,DownRight[0]);
            if((Island_State==5)&&(Boundry_Start_Right<UVC_H-10)&&abs(icmdata.huandao_yaw)>=290)//右边先丢线||DownRightFlag==0
            {
                Island_State=6;
            }
            //Longest_White_Column();
           //Longest_White_Column(0);
           Longest_White_Column(0);
            Get_Mid();
           for(i=10;i<59;i++)
                   {   if(Mid_Line[i]>=40)
                       Mid_Line[i]=40;

                   }
        }
         if(Island_State==6)//还在出
        {
            //DownRightFlag=GetDownRight(80,15);
            //K_Add_Boundry_Right(k,island_state_5_down[1],island_state_5_down[0],0);
             
            if((Island_State==6)&&(abs(icmdata.huandao_yaw)>=330))//&&(Boundry_Start_Right>=UVC_H-20))//&&((Boundry_Start_Right>UVC_H-10)))//||(abs(icmdata.huandao_yaw)>=310)))//右边不丢线||(abs(FJ_Angle)>=320)(Boundry_Start_Right>UVC_H-5)||
            {
                k_L=0;
                Island_State=7;
            }
            //Draw_Line(30,Get_High(30),93,59);
             //Longest_White_Column(0);
              //Draw_Line(30,Get_High(30),79,59);
            //  Longest_White_Column(0);
            //  Get_Mid();
            //  for(i=0;i<=59;i++)
            // {

            //     Mid_Line[i]=Right_Line[i]-Standard_Road_Wide[i]/2;
            // }
            Draw_Line(25,Get_High(25),DownRight[1]+2,DownRight[0]);
            Longest_White_Column(0);
            Get_Mid();
            for(i=10;i<=59;i++)
                   {   if(Mid_Line[i]>=40)
                       Mid_Line[i]=40;

                   }
        }
        if(Island_State==7)//基本出去了，在寻找拐点，准备离开环岛状态
        {
             /* Left_Up_Guai[0]=Find_Left_up_Point(5,UVC_H-10);//获取左上点坐标，坐标点合理去8
             Left_Up_Guai[1]=Left_Line[Left_Up_Guai[0]]; */
             UpLeftFlag=GetUpLeft(10,UVC_H-5);
             //huandaozuoup();
            if((Island_State==7)&&(abs(icmdata.huandao_yaw)>=340)&&(((UpLeft[1]<=70)&&(10<=UpLeft[0]&&UpLeft[0]<=UVC_H-5))))//&&abs(icmdata.huandao_yaw)>=340))//注意这里，对横纵坐标都有要求
            {
                Island_State=8;//基本上找到拐点就去8zuoup==1&&
            }
            // for(i=0;i<=59;i++)
            // {

            //     Mid_Line[i]=Right_Line[i]-Standard_Road_Wide[i]/2;
            // }
          
                        for(i=0;i<=59;i++)
            {

                Mid_Line[i]=Right_Line[i]-Standard_Road_Wide[i]/2;
            }
                     
										
                    for(i=10;i<=59;i++)
                   {   if(Mid_Line[i]>=40)
                       Mid_Line[i]=40;

                   }
        }
        if(Island_State==8)//连线，出环最后一步
        {
             /* Left_Up_Guai[0]=Find_Left_up_Point(5,UVC_H-1);//获取左上点坐标
             Left_Up_Guai[1]=Left_Line[Left_Up_Guai[0]]; */
             UpLeftFlag=GetUpLeft(10,UVC_H-5);
            //Lengthen_Left_Boundry(UpLeft[0]-1,UVC_H-1);
//            for(i=0;i<=59;i++)
//            {

//                Mid_Line[i]=Right_Line[i]-Standard_Road_Wide[i]/2;
//            }
					
                    
                        for(i=0;i<=59;i++)
            {

                Mid_Line[i]=Right_Line[i]-Standard_Road_Wide[i]/2;
            }
                     
            
            if((Island_State==8)&&(UpLeftFlag==0)&&(motor.huandao_L6>=70))//当拐点靠下时候，认为出环了，环岛结束UpLeft[0]>=UVC_H-20||(UpLeftFlag==0&&||motor.huandao_L6>=110
            {//要么拐点靠下，要么拐点丢了，切下方不丢线，认为环岛结束了&&(motor.huandao_L6>=40)
                //FJ_Angle=0;//数据清零(UpLeftFlag==0)||(UpLeftFlag==0)
                Island_State=9;//8时候环岛基本结束了，为了防止连续判环，8后会进9，大概几十毫秒后归零，
                Left_Island_Flag=0;
            }
            //编码器积分未加
        }
    }
    

     if(Right_Island_Flag==1)
    {   //printf("Right_Island");
        if(Island_State==1)//1状态下拐点还在，没丢线
        {   //Zebra_Stripes_Flag==2;
            // monotonicity_change_line[0]=Monotonicity_Change_Right(30,5);//单调性改变
            // monotonicity_change_line[1]=Right_Line[monotonicity_change_line[0]];
           // Right_Add_Line((int)(UVC_W-1-(monotonicity_change_line[1]*0.15)),UVC_H-1,monotonicity_change_line[1],monotonicity_change_line[0]);
            for(i=0;i<=UVC_H-1;i++)
            {
               Mid_Line[i] =Left_Line[i]+Standard_Road_Wide[i]/2;
                
            } 
           if(Island_State == 1&&Boundry_Start_Right<30)//右下角先丢线
            {
                Island_State=2;
            }
        }
         if(Island_State==2)//2状态下方丢线，上方即将出现大弧线
        {
            /* monotonicity_change_line[0]=Monotonicity_Change_Right(60,10);//单调性改变
            monotonicity_change_line[1]=Right_Line[monotonicity_change_line[0]]; */
           // Right_Add_Line((int)(UVC_W-1-(monotonicity_change_line[1]*0.15)),UVC_H-1,monotonicity_change_line[1],monotonicity_change_line[0]);
          for(i=0;i<=UVC_H-1;i++)
            {
               Mid_Line[i] =Left_Line[i]+Standard_Road_Wide[i]/2;
                
            }    
            //printf("%d\r\n",Boundry_Start_Right);
           
           if(Island_State==2&&(Boundry_Start_Right>=UVC_H-5)&&motor.huandao_L2>25)//右下角再不丢线进3||monotonicity_change_line[0]>45&&motor.huandao_L2>25

            {
                Island_State=3;//下方丢线，说明大弧线已经下来了
                Right_Island_Flag=1;
            }
           
        }
         if(Island_State==3)//下面已经出现大弧线，且上方出现角点
        {   
            UpRightFlag=GetUpRight(10,50);
          if(UpRightFlag==1)  
					{
						if((10<=UpRight[0]&&UpRight[0]<50)&&(10<UpRight[1]&&UpRight[1]<70))//找第一个符合条件的角点，连线
            {	//此处参数要重写                   
                //Draw_Line(0, 59, UpRight[1] , UpRight[0] );
                 //Longest_White_Column(2);
               
                 Draw_Line(12, 59, UpRight[1] , UpRight[0] );
                 Longest_White_Column(2);
                Get_Mid();
            }
					}
					if(UpRightFlag==0)
					{
						if((10<=UpRight[0]&&UpRight[0]<50)&&(10<UpRight[1]&&UpRight[1]<70))//找第一个符合条件的角点，连线
            {	//此处参数要重写                   
                Draw_Line(12, 59, UpRight[1]+5 , UpRight[0]-5 );
                 Longest_White_Column(2);
                 //Longest_White_Column(2);
                Get_Mid();
                /* for (i = 0; i <= 20; i++)
                {
                    if (Mid_Line[i] <= 80)
                        Mid_Line[i] = 80;

                } */
                //UpRightFlag=GetUpRight(5,55);
            }
					
					}


           
            // if(k_R!=0)//将角点与下方连接，画一条死线
            // {
            //     K_Draw_Line(k_R,40,UVC_H-1,0);
            //     Longest_White_Column_Right_huan();//刷新最长白列
            //     Get_Mid();
            //     for (i = 10; i <= 20; i++)
            //     {
            //       if (Mid_Line[i] <= 85)
            //             {Mid_Line[i] = 85;}

            //     }
            // }
            // else
            // {
            //     /* Right_Up_Guai[0]=Find_Right_up_Point(5,45);//找右上拐点
            //     Right_Up_Guai[1]=Right_Line[Right_Up_Guai[0]]; */
            //     //UpRightFlag=GetUpRight(7,55);

            //     // if(Right_Up_Guai[0]<5)//这里改过，此处为了防止环岛误判，如果经常出现环岛3归零，请修改此处判断条件
            //     // {
            //     //     Island_State=0;
            //     //     Right_Island_Flag=0;
            //     // }

            //     if(k_R==0&&(7<=UpRight[0]&&UpRight[0]<50)&&(40<UpRight[1]&&UpRight[1]<125))//找第一个符合条件的角点，连线
            //     {
            //         island_state_3_up[0]= UpRight[0]+5;
            //         island_state_3_up[1]= UpRight[1]+5;

            //        k_R=(float)((float)(UVC_H-island_state_3_up[0])/(float)(40-island_state_3_up[1]));
            //         K_Draw_Line(k_R,40,UVC_H-1,0);
            //       //  Draw_Line(100,0,20,60);
            //         //Longest_White_Column();//刷新赛道数据
            //           Longest_White_Column_Right_huan();
            //     Get_Mid();
            //     for (i = 10; i <= 20; i++)
            //     {
            //         if (Mid_Line[i] <= 85)
            //             Mid_Line[i] = 85;

            //     }
            // }
            //     }
            
        
            if((Island_State==3)&&(abs(icmdata.huandao_yaw)>=45)&&(abs(icmdata.huandao_yaw)<=100))//只依靠陀螺仪积分
            {
                k_R=0;//斜率清零
                Island_State=4;
               /*  Longest_White_Column();//最长白列
                Get_Mid(); */
            }//记得去最长白列那边改一下，区分下左右环岛
        }

         if(Island_State==4)//4状态完全进去环岛了
        {  
					//for (i = 10; i <= 59; i++)
//                {
//                  if (Mid_Line[i] <= 47)
//                        {Mid_Line[i] = 47;}

//                }
            //环岛积分200度后再打开单调转折判断
            
                
                //   monotonicity_change_line[0]=Monotonicity_Change_Left(UVC_H-10,10);//单调性改变
                // monotonicity_change_line[1]=Left_Line[monotonicity_change_line[0]];
               //printf("%d/r/n",monotonicity_change_line[0]);
               DownLeftFlag=GetDownLeft(58,10);

                if((Island_State==4)&&(10<=DownLeft[0]&&DownLeft[0]<=55&&DownLeft[1]<UVC_W-20)&&abs(icmdata.huandao_yaw)>240)//单调点靠下，进去5
                {//monotonicity_change_line[1]<=120&&
                //    island_state_5_down[0]=UVC_H-1;
                //   island_state_5_down[1]=Left_Line[UVC_H-1]-5;//抓住第一次出现的斜率，定死
                //    k=(float)((float)(UVC_H-monotonicity_change_line[0])/(float)(island_state_5_down[1]-monotonicity_change_line[1]));
                 // K_Add_Boundry_Left(k,island_state_5_down[1],island_state_5_down[0],0);
                 /*  for(i=0;i<=UVC_H-1;i++)
            {
               Left_Line[i] =Right_Line[i]-Standard_Road_Wide[i];
                
            } */Island_State=5;
           /* Draw_Line(135,Get_High(135),DownLeft[1]-2,DownLeft[0]);
              Longest_White_Column_Right_huan();
            Get_Mid();
            for (i = 0; i <= 20; i++)
                {
                  if (Mid_Line[i] <= 85)
                        {Mid_Line[i] = 85;}

                } */
            
                   
                }
            
            
        }
         if(Island_State==5)//准备出环岛
        {    /* for(i=0;i<=UVC_H-1;i++)
            {
               Left_Line[i] =Right_Line[i]-Standard_Road_Wide[i];
                
            } */
            DownLeftFlag=GetDownLeft(58,10);
            Draw_Line(54,Get_High(54),DownLeft[1]-2,DownLeft[0]);
            
            //K_Add_Boundry_Left(k,island_state_5_down[1],island_state_5_down[0],0);
            if(Island_State==5&&Boundry_Start_Left<=UVC_H-10&&abs(icmdata.huandao_yaw)>=290)//左边先丢线
            {
                Island_State=6;
            }
            Longest_White_Column(0);
            Get_Mid();
           for (i = 10; i <= 59; i++)
           {
               if (Mid_Line[i] <= 40)
                   Mid_Line[i] = 40;

           }
        }
        if(Island_State==6)//继续出
        {
            //K_Add_Boundry_Left(k,island_state_5_down[1],island_state_5_down[0],0);
            //DownLeftFlag=GetDownLeft(55,10);
            if(Island_State==6)//左边先丢线||abs(FJ_Angle)>=320)(Boundry_Start_Left>UVC_H-5)||Boundry_Start_Left>UVC_H-5||
            {if(abs(icmdata.huandao_yaw)>=330)//&&Boundry_Start_Left>=UVC_H-20)
            {   k_R=0;
                Island_State=7;
            }
            }
            // for(i=0;i<=59;i++)
            // {

            //     Mid_Line[i]=Left_Line[i]+Standard_Road_Wide[i]/2;
            // }
             Draw_Line(59,Get_High(59),DownLeft[1]-2,DownLeft[0]);
             Longest_White_Column(0);
            Get_Mid();
            //   Draw_Line(49,Get_High(49),0,59);//此处参数需要修改
            //   Longest_White_Column(0);
            //      Get_Mid();

               for (i = 10; i <= 59; i++)
               {
                   if (Mid_Line[i] <= 40)
                       Mid_Line[i] = 40;

               }
        }

         if(Island_State==7)//基本出环岛，找角点
        {
            // Right_Up_Guai[0]=Find_Right_up_Point(5,UVC_H-10);//获取右上点坐标，找到了去8
            // Right_Up_Guai[1]=Right_Line[Right_Up_Guai[0]];
            UpRightFlag=GetUpRight(10,UVC_H-5);
            //huandaoyouup();
            if((Island_State==7)&&(((UpRight[1]>=10&&10<UpRight[0]&&UpRight[0]<=UVC_H-5))&&abs(icmdata.huandao_yaw)>=340))//注意这里，对横纵坐标都有要求，因为赛道不一样，会意外出现拐点
            {//当角点位置合理时，进8||abs(icmdata.huandao_yaw)>=170youup==1&&
                //UpRight[1]>=UVC_W-80
							//此处未加编码器积分
                Island_State=8;
            }  
            
                for(i=0;i<=59;i++)
            {

                Mid_Line[i]=Left_Line[i]+Standard_Road_Wide[i]/2;
            }
            			 //Get_Mid_2();
           
           
           for (i = 10; i <= 59; i++)
               {
                   if (Mid_Line[i] <= 40)
                       Mid_Line[i] = 40;

               }
        }
         if(Island_State==8)//环岛8
        {
            // Right_Up_Guai[0]=Find_Right_up_Point(5,UVC_H-10);//获取右上点坐标
            // Right_Up_Guai[1]=Right_Line[Right_Up_Guai[0]];
           // Lengthen_Right_Boundry(Right_Up_Guai[0]-1,UVC_H-1);
           UpRightFlag=GetUpRight(20,UVC_H-5);
//            for(i=0;i<=59;i++)
//            {

//                Mid_Line[i]=Left_Line[i]+Standard_Road_Wide[i]/2;
//            }
					
          
            
                for(i=0;i<=59;i++)
            {

                Mid_Line[i]=Left_Line[i]+Standard_Road_Wide[i]/2;
            }
            
           if((Island_State==8)&&((UpRightFlag==0)&&(motor.huandao_L6>=70)))//当拐点靠下时候，认为出环了，环岛结束UpRight[0]>=UVC_H-5||(UpRightFlag==0&&
            {//角点靠下，或者下端不丢线，认为出环了||huandao_L6>=
                //FJ_Angle=0;
							//此处未加编码器积分
                Island_State=9;
                Right_Island_Flag=0;
            }
        }
    }
        if(Island_State==9)
        {
            continuity_change_left_flag=Continuity_Change_Left(UVC_H-1-10,20);//连续性判断
        continuity_change_right_flag=Continuity_Change_Right(UVC_H-1-10,20);

        }
    if(Island_State==9&&(continuity_change_left_flag==0&&continuity_change_right_flag==0))
    {

        Island_State=0;
    }
}

/*********斑马线********/
void getchangepoint(int startY, int endY)// startY> endY    Image_Use2[i][j]
{
int i,j,q;
for(q=0;q<=59;q++)
{
    changepoint[q]=0;
    changepointhang[q]=0;
}


for(i=endY;i<=startY;i++)
{
  for(j=19;j<=60;j++)
  {

    if( image_two_value[i][j]==0&&image_two_value[i][j+1]==255)
    {
        changepoint[i]++;
       // changepointhang[i]=i;
    }
  }
}

}




void banmaxian(void)
{
    int i,j=0;
    if(Cross_Flag!=0||banmaxianFlag!=0||Island_State!=0
       )//元素互斥，不是十字，不是，不是坡道，不是停车Stop_Flag!=0 ||Electromagnet_Flag!=0||Barricade_Flag!=0||(1<=Ramp_Flag&&Ramp_Flag<=3)||
    {
        return;
    }
    getchangepoint(55,45);
    for( i=55;i>=45;i--)
    {
    if(changepoint[i]>=4
            &&Road_Wide[i]<=15)
    {
     j++;

    }
    }

    if(banmaxianFlag==0&&j>=3)
    {
        banmaxianFlag=1;
       
    }
  
      }


void chujie()
{
    static const uint8 check_cols[12] = {0,7,14,21,28,35,42,49,56,63,70,77};
    uint8 black_line_count = 0;
    int8 i,j;uint8 black_pixel_count;
    for ( i = 59; i >= 52; i--)  // 图像高度60，从第59行扫描到第52行
    {
         black_pixel_count = 0;
        
        for ( j = 0; j < 12; j++)
        {
            if (image_two_value[i][check_cols[j]] == IMG_BLACK)
                black_pixel_count++;
        }
        
        if (black_pixel_count >=11)  // 12个点中10个为黑（约83%）
        {
            if (++black_line_count >= 5)  // 直接递增并判断
            {
                Chujie_Flag = 1;
                return;
            }
        }
    }
    
    
}

/*砖头*/

int8 Leftzhuantoudown[2]={0};
int8 Rightzhuantoudown[2]={0};
int8 Leftzhuantouup[2]={0};
int8 Rightzhuantouup[2]={0};
int8 LeftzhuantoudownFlag=0;
int8 RightzhuantoudownFlag=0;
int8 LeftzhuantouupFlag=0;
int8 RightzhuantouupFlag=0;
int8 LeftzhuantouFlag=0;
int8 RightzhuantouFlag=0;
int8 zhuanwupan=0;

int saikuanbianhua(int8 y1,int8 y2)
{
    int16 ALL;
    ALL=0;
    for(int y=y1;y>y2;y--)

    {
        if(Standard_Road_Wide[y]-Road_Wide[y]>=5)
        {
            ALL++;
        }
    }
return ALL;
}


int GetzhuantouDownLeft(int y1,int y2)
{
    uint8 findl_x = 0;    //左点
    uint8 findl_y = 0;
    uint8 examl_x = 0;
    uint8 examl_y = 0;
    uint8 examcount = 0;
    int directionlu[5][2] =  {{1,0}, {1,1}, {0,1}, {-1,1}, {-1,1}};  //逆时针上方向数组
    if(LeftzhuantoudownFlag==1)
       LeftzhuantoudownFlag=0;
    for(uint8 y=y1;y>=y2;y--)
        {

            if(LeftzhuantoudownFlag==0 )
            {
                if(Left_Lost_Flag[y]==1)
                {
                //gpio_set_level(BEEP_PIN,1);
                findl_x=Left_Line[y];
                findl_y=y;

                for(uint8 dircount = 0;dircount<5;dircount++)
                                       {
                                           examl_x=findl_x+directionlu[dircount][0];
                                           examl_y=findl_y+directionlu[dircount][1];
                                           if(image_two_value[examl_y][examl_x]==255)
                                           {
                                               examcount++;
                                           }
                                       }
                                       if(examcount>=4&&(Left_Line[findl_y]-Left_Line[findl_y+1]>=5||Left_Line[findl_y]-Left_Line[findl_y+2]>=5))
                                       {
                                           /*if(abs(Left_Line[findl_y]-Left_Line[findl_y+1])>=5&&Road_Wide[findl_y-2]>20)
                                           {*/
                                           examcount=0;
                                           Leftzhuantoudown[0]=findl_y;
                                           Leftzhuantoudown[1]=findl_x;
                                           LeftzhuantoudownFlag=1;
                                         break;
                                           //}
                                       }
                                       else
                                        {
                                           LeftzhuantoudownFlag = 0;
                                            examcount=0;
                                        }
                }

            }

        }

return LeftzhuantoudownFlag;
}

int GetzhuantouUpLeft(int y1,int y2)
{
    uint8 findl_x = 0;    //左点
    uint8 findl_y = 0;
    uint8 examl_x = 0;
    uint8 examl_y = 0;
    uint8 examcount = 0;
    int directionld[5][2] =  {{1,0}, {1,-1}, {0,-1}, {-1,-1}, {-1,-1}};
    if(LeftzhuantouupFlag==1)
            LeftzhuantouupFlag=0;
    if(y1<y2)
    {
    for(uint8 y=y1;y<=y2;y++)
          {

              if(LeftzhuantouupFlag==0 )
              {
                  if(Left_Lost_Flag[y]==1)
                  {
                  //gpio_set_level(BEEP_PIN,1);
                  findl_x=Left_Line[y];
                  findl_y=y;

                  for(uint8 dircount = 0;dircount<5;dircount++)
                                         {
                                             examl_x=findl_x+directionld[dircount][0];
                                             examl_y=findl_y+directionld[dircount][1];
                                             if(image_two_value[examl_y][examl_x]==255)
                                             {
                                                 examcount++;
                                             }
                                         }
                                         if(examcount>=4&&(Left_Line[findl_y]-Left_Line[findl_y-1]>=5||Left_Line[findl_y]-Left_Line[findl_y-2]>=5))
                                         {
                                             examcount=0;
                                             Leftzhuantouup[0]=findl_y;
                                             Leftzhuantouup[1]=findl_x;
                                             LeftzhuantouupFlag=1;
                                           break;
                                         }
                                         else
                                          {
                                             LeftzhuantouupFlag = 0;
                                              examcount=0;
                                          }
                  }

              }

          }
    }
    if(y1>y2)
        {
        for(uint8 y=y1;y>=y2;y--)
              {

                  if(LeftzhuantouupFlag==0 )
                  {
                      if(Left_Lost_Flag[y]==1)
                      {
                      //gpio_set_level(BEEP_PIN,1);
                      findl_x=Left_Line[y];
                      findl_y=y;

                      for(uint8 dircount = 0;dircount<5;dircount++)
                                             {
                                                 examl_x=findl_x+directionld[dircount][0];
                                                 examl_y=findl_y+directionld[dircount][1];
                                                 if(image_two_value[examl_y][examl_x]==255)
                                                 {
                                                     examcount++;
                                                 }
                                             }
                                             if(examcount>=4)
                                             {
                                                 examcount=0;
                                                 Leftzhuantouup[0]=findl_y;
                                                 Leftzhuantouup[1]=findl_x;
                                                 LeftzhuantouupFlag=1;
                                               break;
                                             }
                                             else
                                              {
                                                 LeftzhuantouupFlag = 0;
                                                  examcount=0;
                                              }
                      }

                  }

              }
        }
    return LeftzhuantouupFlag;
}




void Leftzhuantou(void)//White_Column
{   
   uint8 y;
        if(LeftzhuantouFlag==0&&Cross_Flag==0&&Island_State==0&&RightzhuantouFlag==0&&banmaxianFlag==0)
        {   
            if(((GetzhuantouDownLeft(50,20)==1&&GetzhuantouUpLeft(59,5)==1))
                   &&Continuity_Change_Right(58,20)==0&&Left_Lost_Time<5
                   &&Search_Stop_Line>=45&&White_Column[40]>45&&Continuity_Change_Left(58,10)!=0&&
                   abs(Continuity_Change_Left(58,10)-Leftzhuantoudown[0])<=3&&(Left_Line[ Leftzhuantoudown[0]]>Left_Line[ Leftzhuantoudown[0]+1]||Left_Line[ Leftzhuantoudown[0]]>Left_Line[ Leftzhuantoudown[0]+2])&&
                   (Left_Line[ Leftzhuantouup[0]]>Left_Line[ Leftzhuantouup[0]-1]||Left_Line[ Leftzhuantouup[0]]>Left_Line[ Leftzhuantouup[0]-2])
                   &&abs(saikuanbianhua(Leftzhuantoudown[0],Leftzhuantouup[0])-(Leftzhuantoudown[0]-Leftzhuantouup[0]))<=5)//&&(Left_Line[ Leftzhuantoudown[0]]>Left_Line[ Leftzhuantoudown[0]+1]||Left_Line[ Leftzhuantoudown[0]]>Left_Line[ Leftzhuantoudown[0]+2])&&
                   //(Left_Line[ Leftzhuantouup[0]]>Left_Line[ Leftzhuantouup[0]-1]||Left_Line[ Leftzhuantouup[0]]>Left_Line[ Leftzhuantouup[0]-2])

    {
        LeftzhuantouFlag=1;
        

    }}
    /*  */

    if(LeftzhuantouFlag==1&&GetzhuantouDownLeft(59,Leftzhuantouup[0])==0)
    {

        LeftzhuantouFlag=2;

    }

    if(LeftzhuantouFlag==2&&motor.zuozhuantou_L>=40)
    {
        LeftzhuantouFlag=0;
        /* gpio_set_level(BEEP_PIN,0);
        gpio_set_level(LEDR,0);
                  gpio_set_level(LEDG,0);
                  gpio_set_level(LEDB,0); */

    }

    if( LeftzhuantouFlag==1|| LeftzhuantouFlag==2)
    {
        for(y=UVC_H-Search_Stop_Line;y<=59;y++)//UVC_H-1-Search_Stop_Line
                                               {

                                                Mid_Line[y] =Right_Line[y]-Standard_Road_Wide[y]/4;
                                                //Mid_Line[y] =Mid_Line[y]+10;
                                                Mid_Line[y]=limit_a_b(Mid_Line[y],0,79);
                                               }
    }
    /* if(LeftzhuantouFlag!=0)
    {
       if( Continuity_Change_Left(58,5)!=0&& Continuity_Change_Right(58,5)!=0)
        {
            LeftzhuantouFlag=0;
        }

    } */
}


int GetzhuantouDownRight(int y1,int y2)
{
    uint8 findl_x = 0;    //左点
    uint8 findl_y = 0;
    uint8 examl_x = 0;
    uint8 examl_y = 0;
    uint8 examcount = 0;
    int directionlu[5][2] =  {{-1,0}, {-1,1}, {0,1}, {1,1}, {1,1}};  //逆时针上方向数组
    if(RightzhuantoudownFlag==1)
        RightzhuantoudownFlag=0;
    for(uint8 y=y1;y>=y2;y--)
        {

            if(RightzhuantoudownFlag==0 )
            {
                if(Right_Lost_Flag[y]==1)
                {
                //gpio_set_level(BEEP_PIN,1);
                findl_x=Right_Line[y];
                findl_y=y;

                for(uint8 dircount = 0;dircount<5;dircount++)
                                       {
                                           examl_x=findl_x+directionlu[dircount][0];
                                           examl_y=findl_y+directionlu[dircount][1];
                                           if(image_two_value[examl_y][examl_x]==255)
                                           {
                                               examcount++;
                                           }
                                       }
                                       if(examcount>=4&&(Right_Line[findl_y+1]-Right_Line[findl_y]>=5||Right_Line[findl_y+2]-Right_Line[findl_y]>=5))
                                       {
                                           /*if(abs(Left_Line[findl_y]-Left_Line[findl_y+1])>=5&&Road_Wide[findl_y-2]>20)
                                           {*/
                                           examcount=0;
                                           Rightzhuantoudown[0]=findl_y;
                                           Rightzhuantoudown[1]=findl_x;
                                           RightzhuantoudownFlag=1;
                                         break;
                                           //}
                                       }
                                       else
                                        {
                                           RightzhuantoudownFlag = 0;
                                            examcount=0;
                                        }
                }

            }

        }

return RightzhuantoudownFlag;
}

int GetzhuantouUpRight(int y1,int y2)
{
    uint8 findl_x = 0;    //左点
    uint8 findl_y = 0;
    uint8 examl_x = 0;
    uint8 examl_y = 0;
    uint8 examcount = 0;
    int directionld[5][2] =  {{-1,0}, {-1,-1}, {0,-1}, {1,-1}, {1,-1}};
    if(RightzhuantouupFlag==1)
        RightzhuantouupFlag=0;
    if(y1<y2)
    {
    for(uint8 y=y1;y<=y2;y++)
          {

              if(RightzhuantouupFlag==0 )
              {
                  if(Right_Lost_Flag[y]==1)
                  {
                  //gpio_set_level(BEEP_PIN,1);
                  findl_x=Right_Line[y];
                  findl_y=y;

                  for(uint8 dircount = 0;dircount<5;dircount++)
                                         {
                                             examl_x=findl_x+directionld[dircount][0];
                                             examl_y=findl_y+directionld[dircount][1];
                                             if(image_two_value[examl_y][examl_x]==255)
                                             {
                                                 examcount++;
                                             }
                                         }
                                         if(examcount>=4&&(Right_Line[findl_y-1]-Right_Line[findl_y]>=5||Right_Line[findl_y-2]-Right_Line[findl_y]>=5))
                                         {
                                             examcount=0;
                                             Rightzhuantouup[0]=findl_y;
                                             Rightzhuantouup[1]=findl_x;
                                             RightzhuantouupFlag=1;
                                           break;
                                         }
                                         else
                                          {
                                             RightzhuantouupFlag = 0;
                                              examcount=0;
                                          }
                  }

              }

          }
    }
    if(y1>y2)
        {
        for(uint8 y=y1;y>=y2;y--)
              {

                  if(RightzhuantouupFlag==0 )
                  {
                      if(Right_Lost_Flag[y]==1)
                      {
                      //gpio_set_level(BEEP_PIN,1);
                      findl_x=Right_Line[y];
                      findl_y=y;

                      for(uint8 dircount = 0;dircount<5;dircount++)
                                             {
                                                 examl_x=findl_x+directionld[dircount][0];
                                                 examl_y=findl_y+directionld[dircount][1];
                                                 if(image_two_value[examl_y][examl_x]==255)
                                                 {
                                                     examcount++;
                                                 }
                                             }
                                             if(examcount>=4)
                                             {
                                                 examcount=0;
                                                 Rightzhuantouup[0]=findl_y;
                                                 Rightzhuantouup[1]=findl_x;
                                                 RightzhuantouupFlag=1;
                                               break;
                                             }
                                             else
                                              {
                                                 RightzhuantouupFlag = 0;
                                                  examcount=0;
                                              }
                      }

                  }

              }
        }
    return RightzhuantouupFlag;
}

void Rightzhuantou(void)//White_Column
{
    /*RightzhuantouupFlag==1&&RightzhuantoudownFlag==1&&Continuity_Change_Left(58,10)==0
            &&Right_Lost_Time<=10&&Left_Lost_Time<=10&&Both_Lost_Time<=5&&(White_Column[39]<60&&White_Column[39]>40)&&
            (Longest_White_Column_Left[0]<=60&&Longest_White_Column_Left[0]>=40)&&
            (Longest_White_Column_Right[0]<=60&&Longest_White_Column_Right[0]>=40)&&
            Right_Line[ Rightzhuantoudown[1]]<Right_Line[ Rightzhuantoudown[1]+1]&&
            Right_Line[ Rightzhuantouup[1]]<Right_Line[ Rightzhuantoudown[1]-1]&&
            Left_Lost_Flag[Leftzhuantoudown[1]-1]==1&&
            Left_Lost_Flag[Leftzhuantoudown[1]-2]==1&&
            Left_Lost_Flag[Leftzhuantoudown[1]-3]==1&&
            Left_Lost_Flag[Leftzhuantoudown[1]+1]==1&&
            Left_Lost_Flag[Leftzhuantoudown[1]+2]==1&&
            Left_Lost_Flag[Leftzhuantoudown[1]+3]==1&&

            Rightzhuantoudown[1]>Rightzhuantouup[1]+2
            &&abs(Continuity_Change_Right(58,10)-Rightzhuantoudown[1])<=1*/
           if(LeftzhuantouFlag==0&&Cross_Flag==0&&Island_State==0&&RightzhuantouFlag==0&&banmaxianFlag==0)
           { if(((GetzhuantouDownRight(50,20)==1&&GetzhuantouUpRight(59,5)==1))&&
           Continuity_Change_Left(58,20)==0&&Right_Lost_Time<5&&Search_Stop_Line>=45&&White_Column[40]>45
           &&Continuity_Change_Right(58,10)!=0&&abs(Continuity_Change_Right(58,10)-Rightzhuantoudown[0])<=3
           &&(Right_Line[ Rightzhuantoudown[0]]<Right_Line[ Rightzhuantoudown[0]+1]||Right_Line[ Rightzhuantoudown[0]]<Right_Line[ Rightzhuantoudown[0]+2])
           &&(Right_Line[ Rightzhuantouup[0]]<Right_Line[ Rightzhuantouup[0]-1]||Right_Line[ Rightzhuantouup[0]]<Right_Line[ Rightzhuantouup[0]-2])
           &&abs(saikuanbianhua(Rightzhuantoudown[0],Rightzhuantouup[0])-(Rightzhuantoudown[0]-Rightzhuantouup[0]))<=3
          )

            {
        RightzhuantouFlag=1;
       

            }
        }

    if(RightzhuantouFlag==1&&GetzhuantouDownRight(59,Rightzhuantouup[0])==0)
    {

        RightzhuantouFlag=2;
    }

    if(RightzhuantouFlag==2&&motor.youzhuantou_L>=40)
    {
        RightzhuantouFlag=0;
       
    }

    if( RightzhuantouFlag==1|| RightzhuantouFlag==2)
    {
        for(uint8 y=UVC_H-Search_Stop_Line;y<=59;y++)
                                               {

                                                Mid_Line[y] =Left_Line[y]+Standard_Road_Wide[y]/4;
                                           //Mid_Line[y] =Mid_Line[y]-10;
                                                Mid_Line[y]=limit_a_b(Mid_Line[y],0,79);
                                               }
    }
   
}

int8 getchangepoint_zhuan(int startY, int endY)// startY> endY    Image_Use2[i][j]
{
int8 i=0,j=0,q=0,k=0;
for(q=0;q<=59;q++)
{
    changepoint[q]=0;
    changepointhang[q]=0;
}


for(i=endY;i<=startY;i++)
{
  for(j=19;j<=60;j++)
  {

    if( image_two_value[i][j]==0&&image_two_value[i][j+1]==255)
    {
        changepoint[i]++;
       // changepointhang[i]=i;
    }
  }
}
for( i=startY;i>=endY;i--)
    {
    if(changepoint[i]>=4
           )
    {
     k++;

    }
    }
    return k;

}

void zhuantou()
{ 
    Leftzhuantou();
    Rightzhuantou();
}





