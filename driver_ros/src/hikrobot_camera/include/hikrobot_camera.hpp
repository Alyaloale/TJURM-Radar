#ifndef CAMERA_HPP
#define CAMERA_HPP
#include "ros/ros.h"
#include <stdio.h>
#include <pthread.h>
#include <opencv2/opencv.hpp>
#include "MvErrorDefine.h"
#include "CameraParams.h"
#include "MvCameraControl.h"

namespace camera
{
//********** define ************************************/
#define MAX_IMAGE_DATA_SIZE (3 * 2460 * 4096)
// #define MAX_IMAGE_DATA_SIZE (4 * 2048 * 3072)
    //********** frame ************************************/
    cv::Mat frame;
    //********** frame_empty ******************************/
    bool frame_empty = 0;
    //********** mutex ************************************/
    pthread_mutex_t mutex;
    //********** CameraProperties config ************************************/
    enum CamerProperties
    {
        CAP_PROP_FRAMERATE_ENABLE,  //帧数可调
        CAP_PROP_FRAMERATE,         //帧数
        CAP_PROP_BURSTFRAMECOUNT,   //外部一次触发帧数
        CAP_PROP_HEIGHT,            //图像高度
        CAP_PROP_WIDTH,             //图像宽度
        CAP_PROP_EXPOSURE_TIME,     //曝光时间
        CAP_PROP_GAMMA_ENABLE,      //伽马因子可调
        CAP_PROP_GAMMA,             //伽马因子
        CAP_PROP_GAINAUTO,          //亮度
        CAP_PROP_SATURATION_ENABLE, //饱和度可调
        CAP_PROP_SATURATION,        //饱和度
        CAP_PROP_OFFSETX,           //X偏置
        CAP_PROP_OFFSETY,           //Y偏置
        CAP_PROP_TRIGGER_MODE,      //外部触发
        CAP_PROP_TRIGGER_SOURCE,    //触发源
        CAP_PROP_LINE_SELECTOR      //触发线

    };

    //^ *********************************************************************************** //
    //^ ********************************** Camera Class************************************ //
    //^ *********************************************************************************** //
    class Camera
    {
    public:
        //********** 构造函数  ****************************/
        Camera(ros::NodeHandle &node);
        //********** 析构函数  ****************************/
        ~Camera();
        //********** 原始信息转换线程 **********************/
        static void *HKWorkThread(void *p_handle);

        //********** 输出摄像头信息 ***********************/
        bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo);
        //********** 设置摄像头参数 ***********************/
        bool set(camera::CamerProperties type, float value);
        //********** 恢复默认参数 *************************/
        bool reset();
        //********** 读图10个相机的原始图像 ********************************/
        void ReadImg(cv::Mat &image);

    private:
        //********** handle ******************************/
        void *handle;
        //********** nThreadID ******************************/
        pthread_t nThreadID;
        //********** yaml config ******************************/
        int nRet;
        int width;
        int height;
        int Offset_x;
        int Offset_y;
        bool FrameRateEnable;
        int FrameRate;
        int BurstFrameCount;
        int ExposureTime;
        bool GammaEnable;
        float Gamma;
        int GainAuto;
        bool SaturationEnable;
        int Saturation;
        int TriggerMode;
        int TriggerSource;
        int LineSelector;
    };
    //^ *********************************************************************************** //

    //^ ********************************** Camera constructor************************************ //
    Camera::Camera(ros::NodeHandle &node)
    {
        handle = NULL;

        //********** 读取待设置的摄像头参数 第三个参数是默认值 yaml文件未给出该值时生效 ********************************/
        // node.param("width", width, 3072);
        // node.param("height", height, 2048);
        node.param("width", width, 4096);
        node.param("height", height, 2460);
        node.param("FrameRateEnable", FrameRateEnable, false);
        node.param("FrameRate", FrameRate, 10);
        node.param("BurstFrameCount", BurstFrameCount, 10); // 一次触发采集的次数
        node.param("ExposureTime", ExposureTime, 50000);
        node.param("GammaEnable", GammaEnable, false);
        node.param("Gamma", Gamma, (float)0.7);
        node.param("GainAuto", GainAuto, 2);
        node.param("SaturationEnable", SaturationEnable,true);
        node.param("Saturation", Saturation, 128);
        node.param("Offset_x", Offset_x, 0);
        node.param("Offset_y", Offset_y, 0);
        node.param("TriggerMode", TriggerMode, 1);
        node.param("TriggerSource", TriggerSource, 2);
        node.param("LineSelector", LineSelector, 2);

        //********** 枚举设备 ********************************/
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        unsigned int nIndex = 0;
        if (stDeviceList.nDeviceNum > 0)
        {
            for (int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                printf("[device %d]:\n", i);
                MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (NULL == pDeviceInfo)
                {
                    break;
                }
                PrintDeviceInfo(pDeviceInfo);
            }
        }
        else
        {
            printf("Find No Devices!\n");
            exit(-1);
        }

        //********** 选择设备并创建句柄 *************************/

        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);

        if (MV_OK != nRet)
        {
            printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
            exit(-1);
        }

        // 打开设备
        //********** frame **********/

        nRet = MV_CC_OpenDevice(handle);

        if (MV_OK != nRet)
        {
            printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
            exit(-1);
        }

        //设置 yaml 文件里面的配置
        this->set(CAP_PROP_FRAMERATE_ENABLE, FrameRateEnable);
        if (FrameRateEnable)
            this->set(CAP_PROP_FRAMERATE, FrameRate);
        // this->set(CAP_PROP_BURSTFRAMECOUNT, BurstFrameCount);
        this->set(CAP_PROP_HEIGHT, height);
        this->set(CAP_PROP_WIDTH, width);
        this->set(CAP_PROP_OFFSETX, Offset_x);
        this->set(CAP_PROP_OFFSETY, Offset_y);
        this->set(CAP_PROP_EXPOSURE_TIME, ExposureTime);
        // printf("\n%d\n",GammaEnable);
        this->set(CAP_PROP_GAMMA_ENABLE, GammaEnable);
        // printf("\n%d\n",GammaEnable);
        if (GammaEnable)
            this->set(CAP_PROP_GAMMA, Gamma);
        this->set(CAP_PROP_GAINAUTO, GainAuto);
        // this->set(CAP_PROP_TRIGGER_MODE, TriggerMode);
        // this->set(CAP_PROP_TRIGGER_SOURCE, TriggerSource);
        // this->set(CAP_PROP_LINE_SELECTOR, LineSelector);

        //********** frame **********/
        //白平衡 非自适应（给定参数0）
        nRet = MV_CC_SetEnumValue(handle, "BalanceWhiteAuto", 0);
        // //白平衡度
        // int rgb[3] = {1742, 1024, 2371};
        // for (int i = 0; i < 3; i++)
        // {
        //     //********** frame **********/

        //     nRet = MV_CC_SetEnumValue(handle, "BalanceRatioSelector", i);
        //     nRet = MV_CC_SetIntValue(handle, "BalanceRatio", rgb[i]);
        // }
        if (MV_OK == nRet)
        {
            printf("set BalanceRatio OK! value=%f\n",0.0 );
        }
        else
        {
            printf("Set BalanceRatio Failed! nRet = [%x]\n\n", nRet);
        }
        this->set(CAP_PROP_SATURATION_ENABLE, SaturationEnable);
        if (SaturationEnable)
            this->set(CAP_PROP_SATURATION, Saturation);
        //软件触发
        // ********** frame **********/
        nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
        if (MV_OK == nRet)
        {
            printf("set TriggerMode OK!\n");
        }
        else
        {
            printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
        }

        //********** 图像格式 **********/
        // 0x01100003:Mono10
        // 0x010C0004:Mono10Packed
        // 0x01100005:Mono12
        // 0x010C0006:Mono12Packed
        // 0x01100007:Mono16
        // 0x02180014:RGB8Packed
        // 0x02100032:YUV422_8
        // 0x0210001F:YUV422_8_UYVY
        // 0x01080008:BayerGR8
        // 0x01080009:BayerRG8
        // 0x0108000A:BayerGB8
        // 0x0108000B:BayerBG8
        // 0x0110000e:BayerGB10
        // 0x01100012:BayerGB12
        // 0x010C002C:BayerGB12Packed
        nRet = MV_CC_SetEnumValue(handle, "PixelFormat", 0x02180015); // 目前 BGR  

        if (MV_OK == nRet)
        {
            printf("set PixelFormat OK ! value = RGB\n");
        }
        else
        {
            printf("MV_CC_SetPixelFormat fail! nRet [%x]\n", nRet);
            // 获取支持的格式列表
            MVCC_ENUMVALUE stEnumValue = {0};
            MV_CC_GetEnumValue(handle, "PixelFormat", &stEnumValue);
            ROS_INFO("Supported formats: %d types", stEnumValue.nSupportedNum);
            
            for(unsigned int i=0; i<stEnumValue.nSupportedNum; i++){
                ROS_INFO("Format %d: 0x%08X", i, stEnumValue.nSupportValue[i]);
            }
        }
        MVCC_ENUMVALUE t = {0};
        //********** frame **********/

        nRet = MV_CC_GetEnumValue(handle, "PixelFormat", &t);

        if (MV_OK == nRet)
        {
            printf("PixelFormat :%d!\n", t.nCurValue); // 35127316
        }
        else
        {
            printf("get PixelFormat fail! nRet [%x]\n", nRet);
        }
        // 开始取流
        //********** frame **********/

        nRet = MV_CC_StartGrabbing(handle);

        if (MV_OK != nRet)
        {
            printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        //初始化互斥量
        nRet = pthread_mutex_init(&mutex, NULL);
        if (nRet != 0)
        {
            perror("pthread_create failed\n");
            exit(-1);
        }
        //********** frame **********/

        nRet = pthread_create(&nThreadID, NULL, HKWorkThread, handle);

        if (nRet != 0)
        {
            printf("thread create failed.ret = %d\n", nRet);
            exit(-1);
        }
    }

    //^ ********************************** Camera constructor************************************ //
    Camera::~Camera()
    {
        int nRet;
        //********** frame **********/

        pthread_join(nThreadID, NULL);

        //********** frame **********/

        nRet = MV_CC_StopGrabbing(handle);

        if (MV_OK != nRet)
        {
            printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        printf("MV_CC_StopGrabbing succeed.\n");
        // 关闭设备
        //********** frame **********/

        nRet = MV_CC_CloseDevice(handle);

        if (MV_OK != nRet)
        {
            printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        printf("MV_CC_CloseDevice succeed.\n");
        // 销毁句柄
        //********** frame **********/

        nRet = MV_CC_DestroyHandle(handle);

        if (MV_OK != nRet)
        {
            printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        printf("MV_CC_DestroyHandle succeed.\n");
        // 销毁互斥量
        pthread_mutex_destroy(&mutex);
    }

    //^ ********************************** Camera constructor************************************ //
    bool Camera::set(CamerProperties type, float value)
    {
        switch (type)
        {
        case CAP_PROP_FRAMERATE_ENABLE:
        {
            //********** frame **********/

            nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", value);

            if (MV_OK == nRet)
            {
                printf("set AcquisitionFrameRateEnable OK! value=%f\n",value);
            }
            else
            {
                printf("Set AcquisitionFrameRateEnable Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_FRAMERATE:
        {
            //********** frame **********/

            nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", value);

            if (MV_OK == nRet)
            {
                printf("set AcquisitionFrameRate OK! value=%f\n",value);
            }
            else
            {
                printf("Set AcquisitionFrameRate Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_BURSTFRAMECOUNT:
        {
            //********** frame **********/

            nRet = MV_CC_SetIntValue(handle, "AcquisitionBurstFrameCount", value);

            if (MV_OK == nRet)
            {
                printf("set AcquisitionBurstFrameCount OK!\n");
            }
            else
            {
                printf("Set AcquisitionBurstFrameCount Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_HEIGHT:
        {
            //********** frame **********/

            nRet = MV_CC_SetIntValue(handle, "Height", value); //图像高度

            if (MV_OK == nRet)
            {
                printf("set Height OK!\n");
            }
            else
            {
                printf("Set Height Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_WIDTH:
        {
            //********** frame **********/

            nRet = MV_CC_SetIntValue(handle, "Width", value); //图像宽度

            if (MV_OK == nRet)
            {
                printf("set Width OK!\n");
            }
            else
            {
                printf("Set Width Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_OFFSETX:
        {
            //********** frame **********/

            nRet = MV_CC_SetIntValue(handle, "OffsetX", value); //图像宽度

            if (MV_OK == nRet)
            {
                printf("set Offset X OK!\n");
            }
            else
            {
                printf("Set Offset X Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_OFFSETY:
        {
            //********** frame **********/

            nRet = MV_CC_SetIntValue(handle, "OffsetY", value); //图像宽度

            if (MV_OK == nRet)
            {
                printf("set Offset Y OK!\n");
            }
            else
            {
                printf("Set Offset Y Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_EXPOSURE_TIME:
        {
            //********** frame **********/

            nRet = MV_CC_SetFloatValue(handle, "ExposureTime", value); //曝光时间

            if (MV_OK == nRet)
            {
                printf("set ExposureTime OK! value=%f\n",value);
            }
            else
            {
                printf("Set ExposureTime Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_GAMMA_ENABLE:
        {
            //********** frame **********/

            nRet = MV_CC_SetBoolValue(handle, "GammaEnable", value); //伽马因子是否可调  默认不可调（false）

            if (MV_OK == nRet)
            {
                printf("set GammaEnable OK! value=%f\n",value);
            }
            else
            {
                printf("Set GammaEnable Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_GAMMA:
        {
            //********** frame **********/

            nRet = MV_CC_SetFloatValue(handle, "Gamma", value); //伽马越小 亮度越大

            if (MV_OK == nRet)
            {
                printf("set Gamma OK! value=%f\n",value);
            }
            else
            {
                printf("Set Gamma Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_GAINAUTO:
        {
            //********** frame **********/

            nRet = MV_CC_SetEnumValue(handle, "GainAuto", value); //亮度 越大越亮

            if (MV_OK == nRet)
            {
                printf("set GainAuto OK! value=%f\n",value);
            }
            else
            {
                printf("Set GainAuto Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_SATURATION_ENABLE:
        {
            //********** frame **********/

            nRet = MV_CC_SetBoolValue(handle, "SaturationEnable", value); //饱和度是否可调 默认不可调(false)

            if (MV_OK == nRet)
            {
                printf("set SaturationEnable OK! value=%f\n",value);
            }
            else
            {
                printf("Set SaturationEnable Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_SATURATION:
        {
            //********** frame **********/

            nRet = MV_CC_SetIntValue(handle, "Saturation", value); //饱和度 默认128 最大255

            if (MV_OK == nRet)
            {
                printf("set Saturation OK! value=%f\n",value);
            }
            else
            {
                printf("Set Saturation Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }

        case CAP_PROP_TRIGGER_MODE:
        {

            nRet = MV_CC_SetEnumValue(handle, "TriggerMode", value); //饱和度 默认128 最大255

            if (MV_OK == nRet)
            {
                printf("set TriggerMode OK!\n");
            }
            else
            {
                printf("Set TriggerMode Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_TRIGGER_SOURCE:
        {

            nRet = MV_CC_SetEnumValue(handle, "TriggerSource", value); //饱和度 默认128 最大255255

            if (MV_OK == nRet)
            {
                printf("set TriggerSource OK!\n");
            }
            else
            {
                printf("Set TriggerSource Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_LINE_SELECTOR:
        {

            nRet = MV_CC_SetEnumValue(handle, "LineSelector", value); //饱和度 默认128 最大255

            if (MV_OK == nRet)
            {
                printf("set LineSelector OK!\n");
            }
            else
            {
                printf("Set LineSelector Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }

        default:
            return 0;
        }
        return nRet;
    }

    //^ ********************************** Camera constructor************************************ //
    bool Camera::reset()
    {
        nRet = this->set(CAP_PROP_FRAMERATE_ENABLE, FrameRateEnable);
        nRet = this->set(CAP_PROP_FRAMERATE, FrameRate) || nRet;
        // nRet = this->set(CAP_PROP_BURSTFRAMECOUNT, BurstFrameCount) || nRet;
        nRet = this->set(CAP_PROP_HEIGHT, height) || nRet;
        nRet = this->set(CAP_PROP_WIDTH, width) || nRet;
        nRet = this->set(CAP_PROP_OFFSETX, Offset_x) || nRet;
        nRet = this->set(CAP_PROP_OFFSETY, Offset_y) || nRet;
        nRet = this->set(CAP_PROP_EXPOSURE_TIME, ExposureTime) || nRet;
        nRet = this->set(CAP_PROP_GAMMA_ENABLE, GammaEnable) || nRet;
        nRet = this->set(CAP_PROP_GAMMA, Gamma) || nRet;
        nRet = this->set(CAP_PROP_GAINAUTO, GainAuto) || nRet;
        nRet = this->set(CAP_PROP_SATURATION_ENABLE, SaturationEnable) || nRet;
        nRet = this->set(CAP_PROP_SATURATION, Saturation) || nRet;
        nRet = this->set(CAP_PROP_TRIGGER_MODE, TriggerMode) || nRet;
        nRet = this->set(CAP_PROP_TRIGGER_SOURCE, TriggerSource) || nRet;
        nRet = this->set(CAP_PROP_LINE_SELECTOR, LineSelector) || nRet;
        return nRet;
    }

    //^ ********************************** PrintDeviceInfo ************************************ //
    bool Camera::PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo)
    {
        if (NULL == pstMVDevInfo)
        {
            printf("%s\n", "The Pointer of pstMVDevInfoList is NULL!");
            return false;
        }
        if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
        {
            printf("%s %x\n", "nCurrentIp:", pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp);                 //当前IP
            printf("%s %s\n\n", "chUserDefinedName:", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName); //用户定义名
        }
        else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
        {
            printf("UserDefinedName:%s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
        }
        else
        {
            printf("Not support.\n");
        }
        return true;
    }

    //^ ********************************** Camera constructor************************************ //
    void Camera::ReadImg(cv::Mat &image)
    {

        pthread_mutex_lock(&mutex);
        if (frame_empty)
        {
            image = cv::Mat();
        }
        else
        {
            image = camera::frame.clone();
            frame_empty = 1;
        }
        pthread_mutex_unlock(&mutex);
    }

    //^ ********************************** HKWorkThread1 ************************************ //
    void* Camera::HKWorkThread(void* p_handle) {
        const int buf_size = 4096 * 2460 * 3; // 根据分辨率计算
        unsigned char* pBuf = new unsigned char[buf_size];
        
        while(ros::ok()) {
            MV_FRAME_OUT_INFO_EX stImageInfo = {0};
            
            // 获取原始帧
            int nRet = MV_CC_GetOneFrameTimeout(p_handle, pBuf, buf_size, 
                                              &stImageInfo, 15);
            if(nRet != MV_OK) continue;
    
            // 根据实际格式处理
            cv::Mat outputImg;
            if(stImageInfo.enPixelType == PixelType_Gvsp_BGR8_Packed) {
                // 直接使用BGR格式
                outputImg = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth,
                                  CV_8UC3, pBuf);
            } 
            else if(stImageInfo.enPixelType == PixelType_Gvsp_BayerGR8) {
                // Bayer转BGR
                cv::Mat bayerImg(stImageInfo.nHeight, stImageInfo.nWidth,
                               CV_8UC1, pBuf);
                cv::cvtColor(bayerImg, outputImg, cv::COLOR_BayerGR2BGR);
            }
    
            // 线程安全更新
            pthread_mutex_lock(&mutex);
            camera::frame = outputImg.clone(); // 深拷贝
            frame_empty = false;
            pthread_mutex_unlock(&mutex);
        }
        
        delete[] pBuf;
        return nullptr;
    }

} // namespace camera
#endif
