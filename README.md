Name: Lio_Slam 
Project Description: 
Demo: a pic show the extent of data set [kitti, our data]
      a gif, shows how the trajectory and cloud grows 
      show, how optimization works
      a table shows the numeric metrics

Systematic graph: 
      the design of systems

           ros2 node<mapping> -> io->gps,imu,lidar
                                 front_end -> lio
                                 back_end -> local to global optimization
                                             loop closure  

           ros2 node<visulization>

 Slam pipeline:
        - imu propogation
        - eskf state estimate
        - lidar inc_ndt
        - gps -> utm 
        - local -to global optimization

Evaluation: 
        ???

Environment and library:
      -- c++ 14, ubuntu 20.04
      -- Ros2, foxy
      -- Opengl
      -- Eigen, Sophus
      -- g2o

Dataset: 
    -- Kitti dataset
    -- Our collection dataset

main challendges:
    -- imu missing 
    -- parameter tuning
    -- time synchronization
    -- kitti dataset limitation
    -- heading missing
    -- gps missing
    -- result evaluation
    -- cloud degenracy

future work: 
   -- lio-visual fused slam 
   -- object based plane fitting
   -- vggt, model based
    
    
     
      
