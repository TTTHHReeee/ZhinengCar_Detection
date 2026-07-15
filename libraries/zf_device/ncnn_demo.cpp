#include "ncnn_demo.hpp"

#include <chrono>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

#include "zf_common_headfile.hpp"


// /********************************************************************************
//  * @fn      NcnnClassifier::NcnnClassifier();
//  * @brief   构造函数，初始化模型和触发状态.
//  * @param   none.
//  * @return  none.
//  ********************************************************************************/
NcnnClassifier::NcnnClassifier()
{
}

// /********************************************************************************
//  * @fn      NcnnClassifier::~NcnnClassifier();
//  * @brief   析构函数.
//  * @param   none.
//  * @return  none.
//  ********************************************************************************/
NcnnClassifier::~NcnnClassifier()
{
}

// /********************************************************************************
//  * @fn      bool NcnnClassifier::ncnn_Init(void);
//  * @brief   加载并初始化 NCNN 模型.
//  * @param   none.
//  * @return  true: 成功, false: 失败.
//  ********************************************************************************/
bool NcnnClassifier::ncnn_Init(void)
{
    std::string model_param = "tiny_classifier_fp32.ncnn.param";
    std::string model_bin   = "tiny_classifier_fp32.ncnn.bin";
    int input_width  = 40;
    int input_height = 40;

    //std::vector<std::string> labels = {"see", "Secure", "Police_car", "Secure_car", "boom", "gun"};
    std::vector<std::string> labels = {"Police_car", "Secure", "Secure_car", "boom", "gun", "see"};
    float mean_vals[3] = {127.5f, 127.5f, 127.5f};
    float norm_vals[3] = {0.00784313725f, 0.00784313725f, 0.00784313725f};

    ncnn_.SetModelPath(model_param, model_bin);
    ncnn_.SetInputSize(input_width, input_height);
    ncnn_.SetLabels(labels);
    ncnn_.SetNormalize(mean_vals, norm_vals);

    if (!ncnn_.Init()) {
        printf("[Classifier] 模型加载失败!\n");
        return false;
    }
    ncnn_.SetConfidenceThreshold(0.1f);
    printf("[Classifier] 模型加载成功\n");
    return true;
}

//********************************************************************************
//  * @fn      int NcnnClassifier::MapLabelToId(const std::string &label);
//  * @brief   将类别名称映射为数字编号.
//  * @param   label: 类别名称.
//  * @return  类别编号，-1=未知.
//  ********************************************************************************/
int NcnnClassifier::MapLabelToId(const std::string &label)
{
    auto it = kLabelMap.find(label);
    return (it != kLabelMap.end()) ? it->second : -1;
}

// /********************************************************************************
//  * @fn      int NcnnClassifier::Main_Detection(const cv::Mat &roi_rgb);
//  * @brief   执行 NCNN 推理并通过映射表返回类别编号.
//  * @param   roi_rgb: 50x50 RGB 输入图像.
//  * @return  1=Police_car, 2=boom, 3=gun, -1=低于阈值/未知.
//  ********************************************************************************/
int NcnnClassifier::Main_Detection(const cv::Mat &roi_rgb)
{
    int class_id = 0;
    auto t1 = std::chrono::high_resolution_clock::now();
    auto [label, confidence] = ncnn_.InferRGB(roi_rgb);
    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration =std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
        
        class_id = MapLabelToId(label);
    
    const char *cat ="";
    if (class_id ==1) cat = "supply";
    else if (class_id ==2) cat = "vehicle";
    else if (class_id ==3) cat = "weapon";

    printf("\n========================================\n");
    printf("推理结果: %s (id=%d)\n", cat, class_id);
    printf("推理耗时: %ld ms\n", duration.count());
    printf("置信度: %.2f\n", confidence);
    printf("========================================\n");

    return class_id;

}


