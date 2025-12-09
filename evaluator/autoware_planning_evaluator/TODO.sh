metrics_calculator改造：


先融合再计算复杂度反而更高，回归原始，考虑剪枝

轨迹点的处理挪到最外面，因为要复用，resample节约计算量，calcDistanceToObstacle也要用，果然还是需要一个大函数？


NPC应该最开始就补充一个终点让他终点时间和ego的最后一个点一致，避免一直去计算延长线。

生成轨迹几何还不如原来ttc（N*M，更新点位置本质是O（1）），

找到有轨迹交叉后，

TTC:（从可能交叉点开始顺序找footprint第一个碰撞点）
当前算法但是用优化的footprint。（原ttc因为要用速度，不能resample，我们保留2个函数，然后on/off来选择调用什么函数），


PET：（从可能交叉的开始找第一个重叠事件点），

重叠点，从交叉点前推第一个footprint，有多交叉则return？ 
for循环找各自的前段到该交叉点的累计距离，直接除以速度算时间。

DRAC
然后对交叠的object的footprint的下一个撞不上的footprint，找到这个装不上的点的时间后，算ego花这个时间到前面撞上的footprint需要的减速度。
对EGO的点做这个for循环
交互寻找，ego撞了就for Object直到不撞，然后这个点的时间算ego需要的减速度。然后ego下一个点继续，但要从原来？继续。


原则：
不需要resample!! 也不能cut前后， autoware是0.1s/10s=100个点

ego不必要算fp的微分


先粗筛碰撞可能物体（用距离）

for循环过去 算TTC和DRAC。

碰不到的物体中：找是否有时


总算法：
set 最远点距离ego_max_distance = 0

插入第一个点
如果ego速度小于stop_velocity_mps:
  第一个点的is_stop = True。
  结束函数。


for ego轨迹点：
  如果使用ego轨迹速度，则：
    如果(flag and 当前点不为最后一个点) 则：
        continue
    插入点。（异常停止的情况要用limit_min_accel）。
    如果当前插入点速度为0（此后停止）,立flag说明已到停止点。
    更新is_stop 
  否则：
    生成EgoTrajectoryPoint到最后。
  计算并更新该点到原始位置的最大距离ego_max_distance




算ego_margin_max = baselink到最远点（左前点，左后点，右前点，右后点距离的max）的距离
ego_margin_min = baselink到最近点（左前点，左后点，右前点，右后点距离的min）的距离


for 每个object:
  算 object_margin_max = 中心点到该形状远点的距离（得按shape） + ego_margin_max + parameters.collision_thr_m
  算 object_margin_min = 中心点到该形状近点的距离（得按shape） + ego_margin_min + parameters.collision_thr_m

  算 object到原始ego点的距离 object_distance
  按NPC速度和最后一个EGO点的时间，计算NPC的最大可达距离 object_max_distance

  is_stop = False
  flag_no_overlapping = False
  if object_distance > object_max_distance + ego_max_distance + object_margin，则一定走不到ego轨迹上：
    创单个ObstacleTrajectoryPoint（要用来计算obstacle_distance）放入vector，is_stop = False #这个没关系，因为flag_no_overlapping直接跳过后续计算。
    flag_no_overlapping = True
  elif object速度为在阈值以下：
    创单个ObstacleTrajectoryPoint放入vector，is_stop = True
  else:
    reserve ego数量的ObstacleTrajectoryPoint的内存
      创建N个的ObstacleTrajectoryPoint，每次都要以ego点的时间为参考。
    //// (ego第一个点为is_stop或最后的点为stop的时候不需要计算后续的ttc等metric所以没问题,这个操作保证time_from_start_s一致)



  if 要算obstacle_distance：
    obstacle_distance = max
    for id_ego in range(N):
        if idx_obj == 0:
         计算footprint最小距离
         object_distance = min(object_distance, footprint最小距离)
         碰撞flag = d == 0


  object_ttc_index = -1
  object_first_overlapping_index = -1
  for idx_obj in range(N):
    obstacle_distance = max
    ttc = max
    for idx_ego in range(N):
       碰撞flag = False
       if idx_obj == 0:
         计算footprint最小距离
         object_distance = min(object_distance, footprint最小距离)
         碰撞flag = d == 0
       
       if flag_no_overlapping:
        continue # 直接跳过第一个点的obstacle_distance以外所有metric计算。

       算 点点距离
       if 如果 点点距离 > object_margin，碰撞flag =False
       elif 如果 点点距离 < object_margin_min: 碰撞flag =True
       else:
         碰撞flag =footprint是否碰撞

       if 碰撞flag: # ——————————TODO 思考到这了
         
         if (要计算ttc):#思考ego静止的情况（其实不用考虑），以及思考NPC静止的情况（要考虑）
            if ego.time_from_start_s == object.time_from_start_s or object.is_stop:
              ttc = min(ttc, ego.time_from_start_s)
              is_overlapping_with_ego_trajectory
              if (DRAC):
                
         if (DRAC):
          
      
    填obstacle_distance入metric。