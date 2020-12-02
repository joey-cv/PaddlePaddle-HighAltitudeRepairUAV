# 基于PaddlePaddle的高空补漆巡检无人机
paddle
维源翼团队的代码是在开源代码的基础进行修改、联调后使用的，源代码共分为5个大类：  
1.stm32机械臂控制。负责控制机械臂、泵、打磨头等等。  
2.机械图纸，关于无人机的SolidWorks机械图纸。  
3.vins-fusion，代码由港科大开源，负责视觉slam定位导航。  
4.dji-sdk onboard，大疆A3飞控，负责无人机飞行控制、路线巡检。  
5.PaddleDectection，无人机视觉识别，原先使用的是yolov4算法，后来改用了开源的ppyolo算法。  
  
（其中vins-fusion与dji-sdk onboard是基于ROS框架下的）  
（在此我们只说明ppyolo的使用教程）  
我们原先使用的是yolov4算法，但是前段时间ppyolo的出现，将one-stage识别算法的速率再次提高，针对于我们作品的特性，再加上PaddlePaddle中的PaddleDectection已经为我们写好ppyolo算法，因此我们最终改用PaddleDectection开源算法。  
在这里，我们已经训练好了权重文件，并且已将参数进行多次修改，得到了已经保存好的权重文件保存在 output/ppyolo/pdt/ppyolo.pdparams  
我们已经准备好了测试图片：demo/img，测试视频:demo/video/test2.mp4  
下面是运行的命令  
# 单张图片识别  
CUDA_VISIBLE_DEVICES=0 python tools/infer.py -c configs/ppyolo/ppyolo.yml -o weights=output/ppyolo/pdt/ppyolo.pdparams --infer_img=demo/img/test_100.jpg  

# 目录所有图片识别  
CUDA_VISIBLE_DEVICES=0 python tools/infer.py -c configs/ppyolo/ppyolo.yml -o weights= output/ppyolo/pdt/ppyolo.pdparams --infer_dir=demo/img/  

#视频流识别  
python tools/export_model.py -c configs/ppyolo/ppyolo.yml -o weights= output/ppyolo/pdt/ppyolo.pdparams  
python deploy/python/infer.py --model_dir=output/ppyolo/ --video_file=demo/video/test2.mp4 --use_gpu=True  --thresh=0.2  
  
输出的目录为output。  
  
我们将实际操作过程中的视频放到了根目录的<实际操作视频中>  


