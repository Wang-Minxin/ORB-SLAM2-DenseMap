/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#include "pointcloudmapping.h"
#include <KeyFrame.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "Converter.h"
#include "PointCloude.h"
#include "System.h"

int currentloopcount = 0;
PointCloudMapping::PointCloudMapping(double resolution_,double meank_,double thresh_)
{
    this->resolution = resolution_;
    // this->meank = thresh_;   // this is source ,modified by han 20210313;
    this->meank = meank_;      // write by han 20210313;
    this->thresh = thresh_;
    statistical_filter.setMeanK(meank);
    statistical_filter.setStddevMulThresh(thresh);
    voxel.setLeafSize( resolution, resolution, resolution);
    globalMap = boost::make_shared< PointCloud >( );
    
    viewerThread = make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );  //绑定并启动view线程；
    bStop = false;  //added by han , 20210313;
    loopbusy = false;//added by han , 20210313;
    cloudbusy = false;//added by han , 20210313;
}

void PointCloudMapping::shutdown()
{
    {
        unique_lock<mutex> lck(shutDownMutex);
        shutDownFlag = true;
        keyFrameUpdated.notify_one();
    }
    viewerThread->join();
}

void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth,int idk,vector<KeyFrame*> vpKFs)
{
    // cout<<"receive a keyframe, id = "<<idk<<" 第"<<kf->mnId<<"个"<<endl;
    //cout<<"vpKFs数量"<<vpKFs.size()<<endl;
    unique_lock<mutex> lck(keyframeMutex);
    keyframes.push_back( kf );
    currentvpKFs = vpKFs;
    //colorImgs.push_back( color.clone() );
    //depthImgs.push_back( depth.clone() );
    //获取本帧图像关联的点云　id, 想对于世界坐标系的变换；
    PointCloude* pointcloude = new PointCloude();
    ( *pointcloude).pcID = idk;
    ( *pointcloude).T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
    //联合深度图和RGB图像，找到对应点的空间位置，并且构造空间点云．
    ( *pointcloude).pcE = generatePointCloud(kf,color,depth);
    pointcloud.push_back( (*pointcloude) );
    // pointcloud.insert(pointcloud.end(), pointcloude);
    
    keyFrameUpdated.notify_one();
}

pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)//,Eigen::Isometry3d T
{

    // cout << "before Ptr tmp( new PointCloud() )   " << endl;
    PointCloud::Ptr tmp( new PointCloud() );
    // cout << "after Ptr tmp( new PointCloud() )" << endl;    
    // point cloud is null ptr
     // for ( int m=0; m<depth.rows; m+=3 )                 // source code : rewrite by han; 20210316;;
    for( int m=0; m<depth.rows; m+=1 )   //Han's code ;   
    {
        for ( int n=0; n<depth.cols;   n+=1 )
        {
            float d = depth.ptr<float>(m)[n];
            if (d < 0.01 || d>5)
                continue;
            PointT p;
            p.z = d;
            // cout << "p.z = d :   " << p.z << endl;
            p.x = ( n - kf->cx) * p.z / kf->fx;
            p.y = ( m - kf->cy) * p.z / kf->fy;
            
            p.b = color.ptr<uchar>(m)[n*3];
            p.g = color.ptr<uchar>(m)[n*3+1];
            p.r = color.ptr<uchar>(m)[n*3+2];
                
            tmp->points.push_back(p);
        }
    }
    
    //Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
    //PointCloud::Ptr cloud(new PointCloud);
    //pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());
    //cloud->is_dense = false;
    
    //cout<<"generate point cloud for kf "<<kf->mnId<<", size="<<cloud->points.size()<<endl;
    // if(tmp->points.size() )
    // {
    //     cout << "tmp->points.size()   > 0  ,size :    " << tmp->points.size()  << endl;
    // }
    // else
    // {
    //     cout << "tmp->points.size()  < 0  , size" << tmp->points.size()  << endl;
    // }
    return tmp;
}


void PointCloudMapping::viewer()
{
    pcl::visualization::CloudViewer viewer("viewer");
    cout << "PointCloudMapping::viewer()********************************* " << endl;
    while(1)
    {
        
        {
            unique_lock<mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag)
            {
                cout << " if (shutDownFlag) *************************************************** " << endl;
                break;
            }
        }
        {
            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
            //等待关键帧的更新；
            keyFrameUpdated.wait( lck_keyframeUpdated );
        }
        
        // keyframe is updated 
        size_t N=0;
        {
            unique_lock<mutex> lck( keyframeMutex );
            N = keyframes.size();
        }
        if(loopbusy || bStop)
        {
            if(loopbusy)
            {
                    cout<<"if(loopbusy || bStop) loopbusy *************************************************"<<endl;
            }
            if(bStop)
            {
                     cout<<"if(loopbusy || bStop)  bStop*************************************************"<<endl;   
            }

            continue;
        }
        //cout<<lastKeyframeSize<<"    "<<N<<endl;
        if(lastKeyframeSize == N)
            cloudbusy = false;                    //han 这句没起作用吧?
        //cout<<"待处理点云个数 = "<<N<<endl;
          cloudbusy = true;
          //遍历关键帧序列的每一帧，提取出每一帧中的点云，并且变换到世界坐标系下,最后加入到全局地图中；
        for ( size_t i=lastKeyframeSize; i<N ; i++ )         
        {

          
            PointCloud::Ptr p (new PointCloud);
            //将每一个关键帧中的点云通过刚体变换变换到世界坐标系下；
            pcl::transformPointCloud( *(pointcloud[i].pcE), *p, pointcloud[i].T.inverse().matrix());
            //cout<<"处理好第i个点云"<<i<<endl;
            //将变换后的点云加入到全局地图中；
            *globalMap += *p;
            //PointCloud::Ptr tmp(new PointCloud());
            //voxel.setInputCloud( globalMap );
           // voxel.filter( *tmp );
            //globalMap->swap( *tmp );
           
 
        }
      
        // depth filter and statistical removal 
        PointCloud::Ptr tmp1 ( new PointCloud );

        //全局点云作为输入，通过统计滤波器;
        statistical_filter.setInputCloud(globalMap);
        statistical_filter.filter( *tmp1 );

       //统计滤波 的输出作为体素滤波器的输入，通过滤波的点输出到全局地图中；
        PointCloud::Ptr tmp(new PointCloud());
        voxel.setInputCloud( tmp1 );
        voxel.filter( *globalMap );
        //globalMap->swap( *tmp );
        viewer.showCloud( globalMap );
        // cout<<"viewer() :   show global map, size=****************"<<N<<"   "<<globalMap->points.size()<<endl;
        lastKeyframeSize = N;
        cloudbusy = false;
        //*globalMap = *tmp1;
        
        //if()
        //{
	    
	//}
    }
}
void PointCloudMapping::save()
{
	pcl::io::savePCDFile( "result.pcd", *globalMap );
	cout<<"globalMap save finished"<<endl;
}
void PointCloudMapping::updatecloud()
{
	if(!cloudbusy)
	{
		loopbusy = true;
        //提取每一个关键帧，提取每一帧上面的点云，转换到世界坐标系下；
        PointCloud::Ptr tmp1(new PointCloud);
		for (int i=0;i<currentvpKFs.size();i++)
		{
		    for (int j=0;j<pointcloud.size();j++)
		    {   
				if(pointcloud[j].pcID==currentvpKFs[i]->mnFrameId) 
				{   
					Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat(currentvpKFs[i]->GetPose() );
					PointCloud::Ptr cloud(new PointCloud);
					pcl::transformPointCloud( *pointcloud[j].pcE, *cloud, T.inverse().matrix());
					*tmp1 +=*cloud;

					//cout<<"第pointcloud"<<j<<"与第vpKFs"<<i<<"匹配"<<endl;
					continue;
				}
			}
		}
        cout<<"finishloopmap"<<endl;
        //通过体素滤波器，得到滤波后的点云；
        PointCloud::Ptr tmp2( new PointCloud() );
        voxel.setInputCloud( tmp1 );
        voxel.filter( *tmp2 );
        globalMap->swap( *tmp2 );
        // viewer.showCloud(globalMap);  //han 解注释　20210312;　这里需要viewer 这个窗口进行显示；
        loopbusy = false;
        //cloudbusy = true;
        loopcount++;   //更新完之后，回换次数才增加一次；

        //*globalMap = *tmp1;
	}
}
