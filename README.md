## 使用说明

### 训练集Mode="train"

#### 文件
- 4得到dataSetn文件夹
- 5得到dataSetNpyn文件夹和csv文件
- 6得到dataTrainm
- 7得到2个pickle文件
- 8得到TRAINING_POINT_CLOUD_m.npy

#### 步骤
1. 创建文件夹dataSetn和文件夹dataSetNpyn
2. 将config.py中的basePath改为dataSetn
3. 在config.py中更改超参数
4. 运行autoVehicle.py采集数据
5. 运行dataProcess.py将txt转成npy，运行generateTotalCsv生成csv文件
6. 在config.py中更改trainNum，运行dataTransform.py生成可用的npy文件夹
7. 运行generateTrainingBaseline.py生成train_queries_baseline_m.pickle和
test_queries_baseline_m.pickle
8. 运行generateFastNpy生成TRAINING_POINT_CLOUD_m.npy

### 评估集Mode="eval"

#### 文件
- 4得到dataSetn文件夹
- 5得到dataSetNpyn文件夹和csv文件
- 6得到dataEvalm
- 7得到2个pickle文件

#### 步骤
1. 创建文件夹dataSetn和文件夹dataSetNpyn
2. 将config.py中的basePath改为dataSetn
3. 在config.py中更改超参数
4. 运行autoVehicle.py采集数据
5. 运行dataProcess.py将txt转成npy，运行generateTotalCsv生成csv文件
6. 在config.py中更改evalNum，运行dataTransform.py生成可用的npy文件夹
7. 运行generateTrainingBaseline.py生成train_queries_baseline_m.pickle和
test_queries_baseline_m.pickle
